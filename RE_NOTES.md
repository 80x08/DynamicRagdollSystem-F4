# Fallout 4 ragdoll / physics — RE notes

Reverse-engineering reference for the Havok (hknp + hkb behavior) ragdoll pipeline in **Fallout 4 OG 1.10.163**
(image base `0x140000000`), as used by DynamicRagdollSystem. Addresses are OG; "ID" = Address Library ID
(`version-1-10-163-0.bin`). Project history and patch notes are in `PROJECT.md`.

Everything here was found in Ghidra and confirmed in game unless marked *(inferred)*.

---

## 1. Physics world

| What | Where |
|---|---|
| `bhkWorld` vtable | `0x142E3D568`; slot `0x28` `Update(uint32)` `0x141D6EC00`, slot `0x29` `Init` `0x141D712C0` |
| `bhkWorldM` vtable | `0x142E433C8` (ID 1488354). Exterior worldspaces use this subclass; interiors use `bhkWorld`. Slot `0x28` is not overridden: it also points to `0x141D6EC00` (same for `0x29`), checked from the RTTI-confirmed vtable. Hooking only `bhkWorld` slot `0x28` misses every exterior cell, so hook slot `0x28` on both vtables. |
| `hknpBSWorld` | `bhkWorld::m_worldNP`; `hknpWorld` subclass |
| world write lock | `hknpBSWorld::m_worldLock` at `+0x6D8`, a `BSReadWriteLock` {u32 writerThread, u32 count; `0x80000000` = write}. Recursive for its owner. `try_lock_write` ID 1279453 |
| gravity | `hknpWorld::m_gravity` `+0x5B0` (−9.81 z) |
| scale | 1 physics unit = 69.99 game units (`1 / 0.0142875`) |

`bhkWorld::Update` (main thread) does **not** hold the write lock; take it before writing bodies.

Step loop inside `bhkWorld::Update` `0x141D6EC00` (ID 1395106): takes the world **write** lock (unless TLS `+0x1539`),
then runs N fixed steps: N = global `0x146542C34` (ID 1163732), step length `0x146542C1C` (ID 505537), frame time
`0x146542C2C` (ID 944230), leftover time accumulates in `0x146542C24`. Skipped when the world is disabled or step length is 0.
- Step listeners: `bhkWorld::m_stepListenerA` (`+0x18`, lock `+0x48`). Add with ID 137330 `0x141D72090`
  `(bhkWorld*, bhkIWorldStepListener*)` (write-locks the list, skips duplicates). The list is emptied at the end of every
  stepping `Update`, so the engine re-adds its char proxy / char rigid-body managers at the top of each `Update`.
- `bhkIWorldStepListener` vtable ID 1274460 `0x142DBEDF8`: dtor + 10 slots, defaults are `ret`. Update sets listener
  `+0x08` (deferred release array) to `0x146542C70`. Order: `+0x08/+0x10` once (begin); per step `+0x18(threads, ctx,
  fraction, stepDt)` → listener tasks → `+0x20(fraction, stepDt)` → hknp collide `0x1414BCD90` → `+0x28/+0x30` →
  solve `0x1414BCFC0` → `+0x38/+0x40`; then `+0x48/+0x50` once (end). fraction = i·stepDt/frameTime clamped to 1.
  All on the main thread with the world write lock held (`try_lock_write` ID 1279453 `0x141B10BA0` is recursive for
  the owning thread). DynamicRagdollSystem uses `+0x20`.

Ground ray (verified in Ghidra): CommonLibF4's `bhkPickData` (ctor ID 526783, `HasHit` 1181584, `GetHitFraction`
476687) plus `TESObjectCELL::Pick(bhkPickData&)` `0x1403B4AA0` (ID 434717).
- `SetStartEnd` `0x1400266C0` (ID 747470) takes game units: it multiplies start/end by the runtime world scale global
  `0x1458CEAB0` (0 in the file, set at startup) and stores the ray origin `+0x20` and direction `+0x30`.
- `TESObjectCELL::Pick` picks the cell's world (interior: cell `+0x48`, else the exterior world) and calls
  `bhkWorld::Pick` `0x141D70640`, which takes the world **read** lock (`hknpBSWorld + 0x6D8`) unless TLS `+0x1539` is
  set: do not call it while holding the write lock.
- Filtering: if `bhkPickData::customCollideLayers` (`+0xC8`) is non-zero, the query uses a stack
  `hknpBSCustomCollisionFilter` (vtable ID 1200668): mask at filter `+0x18` is one bit per layer (`mask >> (layer & 0x3F)`),
  and the query's original `m_collisionFilterInfo` moves to filter `+0x20` (its group excludes same-group bodies).
  A body is hit if it has no collision group (upper 16 bits 0, e.g. world geometry), or its group differs and its
  layer bit is set. With the mask 0 the world's normal filter and the query's layer are used.
- DynamicRagdollSystem casts with mask = Static | AnimStatic | Trees | Props | Terrain | Ground | StairHelper, so ragdoll
  parts (grouped, layer 32/33) are never hit.
Writes from inside the behavior graph pre-physics step (below) happen with whatever lock the engine holds there.

### Bodies and motions
- `hknpBody` (`0x90` bytes, in `m_bodyManager.m_bodies`): transform at `+0x00` (rotation columns, translation
  `+0x30`), `m_flags` (`kIsDynamic`), `m_collisionFilterInfo` `+0x44` (low 7 bits = `COL_LAYER`,
  upper 16 = system group), `m_motionId`.
- `hknpMotion` (via `AccessMotion`): orientation quaternion `+0x10`; packed inverse inertia ×3 + inverse mass
  `+0x20` (4 × u16 = upper half of a float); linear velocity `+0x40` (world); angular velocity `+0x50`
  (motion-local; rotate by orientation).
- Constraint table: `m_constraintManager.constraintBuffer`, `0x38`-byte entries {u32 body0, u32 body1, ptr data
  `+0x08`, u32 id `+0x10`, flags byte `+0x16` (bit 2 = disabled)}.
- Constraint data (`entry+0x08`): `hkpRagdollConstraintData` (0x1A0 bytes, vtable ID 760737) and
  `hkpLimitedHingeConstraintData` (0x130, vtable ID 1540231) both start with the set-local-transforms atom at
  `+0x20`: transform A rotation `+0x30`, pivot A `+0x60`; transform B rotation `+0x70`, pivot B `+0xA0` (body-local).
  Seen in their isValid (`0x1419303E0` / `0x14192AD80`). `hkReferencedObject+0x08` = refcount (u16), `+0x0A` = mem
  size (u16, 0xFFFF = class default); the deleting dtor frees through the TLS Havok heap with that size.

### Impulse APIs (wrappers on `hknpBSWorld`, `(world, bodyId, const hkVector4f* impulse)`)
| | ID | Address | Notes |
|---|---|---|---|
| linear | 488591 | `0x141D6D180` | → `hknpWorld::applyLinearImpulse` `0x1414B3E60` (Δv = invMass·J) + activation `0x141D6D9A0` |
| angular | 222853 | `0x141D6D1B0` | world-space angular impulse |

Both are 35-byte functions; DynamicRagdollSystem fingerprints the exact bytes before use. The checks at `world+0x690`
inside are debug thread checks only.

### Motion type and collision layer of an actor's tree
- `bhkUtilFunctions::SetMotionType(NiAVObject* root, hknpMotionPropertiesId::Preset, bool activate, bool force,
  bool)` ID 357289 `0x141D70E90` — tree visitor. `kDynamic` 1, `kKeyframed` 2. The engine's RagdollStart
  handler calls it as `(root, 1, 1, 1, 1)`.
- Set collision layer on a tree: ID 715635 `0x141D8DA30(root, layer)` (visitor `0x141D92520` → `0x141D8F7F0`,
  sets the low 7 bits). The knockdown uses layer 8 (biped) and never restores it.
- Layers seen: live ragdoll 8 (biped, collides with character controllers), death ragdoll 32 (DeadBip),
  `kBipedNoCC` 33 = still hit by projectiles, ignored by character controllers.
- Layer collision table (verified in Ghidra): `bhkCollisionFilter*` at global ID 469495 `0x1458E0AD8` (vtable ID
  599770 `0x142E3ECA8`; its slot 4 `0x141D890B0` filters body pairs through it). Pair check ID 147831 `0x141D88E60
  (filter, infoA, infoB)`: info bit 14 set = never; either group (upper 16 bits) 0 = always; different groups →
  `mask[layerA] >> layerB & 1` with u64 `mask[64]` at filter `+0x1A0` (layer 30 CharController has extra rules via
  bit 15 and filter `+0x3A0`); same group → the mask, then bit 15 (both) = bone-index adjacency (bits 8–12, collide
  unless 1 apart), or for layers 8/32/33 both a per-bone mask at filter `+0x20 + bone*4`.

**Gotcha:** dynamic ragdoll bodies on layer 8 collide with the actor's own character controller; a moving
controller climbs onto its own ragdoll and rises. Use 33 while the ragdoll is dynamic on a living actor.

---

## 2. Finding an actor's ragdoll

```
actor->currentProcess->middleHigh->animationGraphManager   (BSAnimationGraphManager, +0x258 in middleHigh)
  graph (BShkbAnimationGraph) for each manager->graph[i]
    graph+0x208  hkbRagdollDriver*
    graph+0x210  ragdoll interface (hkbnpRagdollInterface)   — driver+0x70 must equal it
    graph+0x390  generator output holder
    graph+0x3B8  bhkWorld* (set when the ragdoll is "added to world")
  interface+0x18  hknpRagdoll* ; ragdoll+0x18 = hknpBSWorld
  ragdoll  +0x20 body-ID array, +0x30 constraint-ID array, +0x40 hkaSkeleton*, +0x48 bone→body map
  skeleton +0x18 parent indices (s16), +0x28 bones
```
Arrays are `hkArray` {ptr, s32 size, u32 capacityAndFlags}. Human ragdoll: 18 bones/bodies, 9 constraint joints
to dynamic parents. Body names via `bhkUtilFunctions::GetAVObjectFromBodyID`: `COM`, `SPINE1`, `SPINE2`,
`Chest`, `*_Thigh`, `*_Calf`, `*_Foot`, `*Arm*`, `*Hand*` (prefix `L`/`R`). `COM` is not an animation bone but it
**is** the ragdoll's root body and plays the pelvis: there is no `Pelvis` body, and the thighs and SPINE1 hang off
COM. A name test for "Pelvis" never matches a human (it left COM, the heaviest body at 18 kg, with no trunk role).
Ragdoll skeleton bone names (string table in live memory, 1.10.163; table order, not necessarily bone indices,
since the log has SPINE1/SPINE2/Chest at bones 2/5/8): `Ragdoll_NPC COM`, `L Thigh`, `R Thigh`, `Spine`, `L Calf`,
`R Calf`, `Spine1`, `L Foot`, `R Foot`, `Spine2`, `L UpperArm`, `Neck`, `R UpperArm`, `L Forearm`, `Head`,
`R Forearm`, `L Hand`, `R Hand`. No pelvis among the 18. Ragdoll `Spine`/`Spine1`/`Spine2` presumably carry body
nodes `SPINE1`/`SPINE2`/`Chest` (inferred from the parent chain, not traced).

Editor IDs of most forms (races included, e.g. `SynthGen1Race`) are not kept in memory: a string search finds
nothing. Look them up in the ESM instead (RACE group, `EDID` subrecord): `SynthGen1Race` 0x0E8D09,
`SynthGen2Race` 0x10BD65, `SynthGen2RaceValentine` 0x2261A4.

A living NPC's ragdoll bodies are **always in the world** (keyframed, for hit detection) even though the
graph is not "added" (graph+0x3B8 = 0, interface not added).

---

## 3. Ragdoll driver and generator output

### driveToPose
`hkbRagdollDriver::driveToPose(driver, float dt, ctx, generatorOutput**)` ID 993992 `0x1416FDE80`.
- Called once per frame from the graph pre-physics update `0x14161C520` (ID 1348502), call at `+0x12F`
  (`0x14161C64F`, `driver = graph+0x208`, `dt` = global `0x146542C2C`). `0x14161C520` has no direct callers;
  it runs only for graphs whose ragdoll is active (graph vtable slot 19, below).
- Also called with dt 0 from graph vtable slot 7 `0x1416271B0` (reset).
- Early-outs: `driver+0xB7 == 0`, output track count < 3, `driver+0x70 == 0`.
- Computes `driver+0xB8` (powered controller active) and `+0xB9` (rigid-body controller active) **inside the
  call** from the output tracks; reading them before the call shows 0.
- "Lock" calls inside (`0x141720910`/`0x141720920`) are empty `ret` stubs.
- Rigid-body/powered controller dispatch: `0x1418A83F0` (ID 660257). Per-body keyframing `0x1418A91E0`
  (hard keyframe: sets linear/angular velocity to reach the target pose in one step, 1/dt).
- Which bodies are hard-keyframed (per-bone record, 0x30 bytes, byte `+0x20`): track 17 value ≥ 1, read only when
  track 17's weight ≥ 1; every bone listed in track 18 (explicit poses, sparse: palette of 0x30 transforms + s8
  bone map; sets `+0x20` and `+0x21` = use that transform as target); and every bone with no track 13 and no track
  15 entry (map −1, or the track off). The motion type of each body is switched (world vfunc `+0xB0`) to match.

### postPhysics
`0x1416FF3A0` (ID 560275), called from `0x14161C6F0` (ID 1414229) at `+0x1EA`. If `+0xB8` or `+0xB9` is set,
blends the ragdoll pose into the output pose (`0x141700E70`, ID 495894) with the track-13/15 weights. This is
what makes physics visible on the skeleton.

### hkbGeneratorOutput tracks
Holder → output; `s32 trackCount` at `+4`; track header *i* at `+0x10 + i·0x10`:
`{s16 capacity, s16 numData, s16 dataOffset, s16 elementSize, f32 onFraction, u8 flags, u8 type}`.
Sparse tracks (flags `0x04`): `capacity` × element palette at `output + dataOffset`, then an s8 bone→palette
map at `dataOffset + align16(capacity·elementSize)`. Flag `0x10` = track invalid.

| # | Meaning | Notes (human NPC: 21 tracks) |
|---|---|---|
| 0 | world-from-model | |
| 1 | extracted motion | |
| 2 | pose | cap 95 |
| 13 | rigid-body ragdoll controls | cap 18, elem `0x40`, storage always present; sets `+0xB9` |
| 14 | blend duration (float) | 0.5 during get-up |
| 15 | powered ragdoll controls | elem `0x20` {maxForce, tau, damping, propRecoveryVel, constRecoveryVel}; sets `+0xB8` |
| 16 | world-from-model mode | elem 8 `{s16 bone[3]; s8 mode}` → copied to `driver+0x50` (mode at `+0x56`) |
| 17 | keyframed bones | per-bone float; ≥ 1.0 keyframes that bone |
| 18 | explicit bone poses | |

`hkbRigidBodyRagdollControlData` (track 13, `0x40`): hierarchyGain, velocityDamping, accelerationGain,
velocityGain, positionGain, positionMaxLinearVelocity, positionMaxAngularVelocity, snapGain,
snapMaxLinearVelocity, snapMaxAngularVelocity, snapMaxLinearDistance, snapMaxAngularDistance, durationToBlend.
Engine values during get-up: `0.17 0 1 0.6 0.1 1.4 1.8 0.25 0.3 0.3 0.03 0.1 | 0.5`. The gains are
fractions **per update** (scale with frame rate).

Powered values: knockdown `maxForce 0` (limp) `tau 0.8 damping 1.0 2.0 1.0`; get-up start `800 0.6 0.95 3 1`.

Get-up (knock state 6) per frame, v0.4.91 probe (one human NPC, explosion knockdown): first frame track 15 weight
1 `maxForce 800`, track 13 empty, track 17 18 bones at FLT_MAX. From ~0.1 s to the end (5.2 s): track 13 weight 1
with the gains above, track 14 0.5 s, track 15 off, **track 17 all 18 bones = 1.0 (every body keyframed)**. So during
the get-up the bodies are placed on the animation; the track 13 gains have no visible effect. driveToPose ran
~15 times/s for that driver, with ~1.5 s gaps.
Tested (v0.4.92–0.4.96): zeroing track 17 for the 12 non-pelvis/leg bones does un-keyframe them by driveToPose's own
rules (no track 18 poses, all bones have a track 13 entry), and scaling the track 13 gains to 0.1 was applied, but
the get-up looked identical to vanilla (arms still clip through walls). So during a get-up the visible pose does not
follow those ragdoll bodies; driving the get-up through the generator output is a dead end. Not traced further:
postPhysics blending during knock state 6, the driver `+0x58` body set that driveToPose skips.

World-from-model modes (exe strings `0x2E641B0`…): USE_OLD 0, USE_INPUT 1, COMPUTE 2, NONE 3, RAGDOLL 4.

**Driving a living NPC's ragdoll:** enroll the graph (§4), make the bodies dynamic, and write track 13
(palette[0] = control data, map all bones to 0, numData 1, onFraction = weight) and track 15 onFraction 0 in a
hook on the `driveToPose` call. The engine then pulls the ragdoll toward the animation and shows it.

---

## 4. Graph / ragdoll interface (adding and removing ragdolls)

`BShkbAnimationGraph` vtable ID 742655 `0x142DBEE58`:

| Slot | Address | What |
|---|---|---|
| 1 | `0x141626C20` | has ragdoll (`graph+0x210 != 0`) |
| 2 | `0x141626C40` | `driver+0x1E` flag |
| 3 | `0x141626C80` | if world set and interface not added: snapshot bodies, reset via slot 7, restore, interface `+0xA0` add |
| 4 | `0x141627080` | if interface added: interface `+0x78` remove; slot 12(0) |
| 5 | `0x141627180` | `graph+0x3B8 = world` |
| 6 | `0x141627190` | driver reset `0x1416FF5A0` |
| 7 | `0x1416271B0` | driveToPose with dt 0 |
| 10 | `0x1416277C0` | empty |
| 19 | `0x141627BD0` | **ragdoll active**: `graph+0x3B8 != 0 && ragdoll && interface->IsAdded()` |

Graph-manager visitors (`(manager, args*)`, loop over graphs):

| Visitor | ID | Per-graph | Use |
|---|---|---|---|
| `0x140D643B0` | 683498 | `0x140821160` → slot 5 | AddRagdollToWorld `{bhkWorld*, bool}` (ForceAddRagdollToWorld uses it) |
| `0x140D64280` | 258136 | `0x140821110` → slot 3 | register ragdoll (knockdown calls this after add) |
| `0x140DD5310` | 385024 | `0x1408210C0` → slot 4 | remove ragdoll (end of knockdown) |
| `0x140DD51E0` | — | `0x140821280` → slot 10 | no-op |
| `0x1401E4420` | 378824 | `0x140821070` → slot 1 | has ragdoll |
| `0x140DD4D70` | 453608 | slot 19 | ragdoll active (movement query) |

`hkbnpRagdollInterface` vtable ID 1488961 `0x142DC2B88`: `+0x20` IsAdded `0x1417363F0`, `+0x78` remove
`0x141736E30`, `+0xA0` add `0x1417370C0` (adds missing bodies, enables constraints, sets `interface+0x58`).

**Important:** registering (slot 3) without removing (slot 4) leaves the actor "ragdolled" forever (it stops
moving). Vanilla never clears `graph+0x3B8`.

### Who asks "ragdoll active" (slot 19) — needed to keep a ragdolled actor moving
- Actor movement update `0x140D8CD80` (ID 737625) via visitor `0x140DD4D70`, call at `+0x3B8` (`0x140D8D138`):
  if true, skips normal movement and zeroes the character-controller velocity (`0x140DF36E0`).
- Graph per-frame update `0x14161D9A0`, call at `0x14161D9F4`: if true, skips writing the frame's root motion
  → `Actor::ApplyMovementDelta` (TESObjectREFR vfunc `0xAD`, Actor `0x140DBCB30`, calls the movement update at
  `0x140DBCCFA`) reuses the last delta (constant velocity: "freeze", "ice skating", climbing).
- `0x14161CB60` at `0x14161CC6D` (pose/bone branch).
- The dispatcher that runs the pre-physics `driveToPose` call. **Must stay true** — answering "no" globally
  turns the controller off (full limp ragdoll).
Answering "no" only at the first two sites (call-site hook / return-address check) keeps the actor moving
while the ragdoll is driven.

---

## 5. Knockdown / get-up flow

- Knock state: `actor+0x130` bits 21–24 (`ActorState::knockState`): 0 normal, 1 explode, 2 explode lead-in,
  3 out, 4 out lead-in, 5 queued, 6 get-up, 7 down, 8 wait for task queue.
- `AIProcess::KnockExplosion` ID 533106 → `0x140D598B0` queues task 10 → `0x140E83A60`: knockState 2, graph int
  var get-up type (`DAT_1458D5248`) = 1, layer 8, AddRagdollToWorld, register, knockdown action
  (default object `+0x358`), explosion impulse. It launches the body hard regardless of magnitude.
- Per-frame knock update `0x140E73F80`: in explode/out (1/3), when the timer at `middleHigh+0x430` (CommonLib
  `bleedoutTimer`, counts down, set on 2→1 / 4→3) runs out OR the body is calm (`0x140DA4880`: first collision
  body in the 3D tree, linear speed² < 0.025, or < 0.15 with actor +0x2D0 bit 18) → knockState 8 and task 0xD
  — unless the actor is dead or Actor+0x2D0 bit 31 is set (= Paralysis > 0; set by the Paralysis AV callback
  0x140DF8180). `0x140E73C10`: a paralyzed actor in knock state 0 or 6 is knocked down (state 3, ragdoll). → `0x140E86810` start get-up (get-up type 1 or 2,
  get-up action, knockState 5).
- Get-up type (graph int var `DAT_1458D5248`), `0x140E86810`: 1 = normal (default). 2 = underwater, only when
  Actor+0x2D0 bit 18 (in water) is set and the body is ≥ 0.875 of the actor height below the water surface:
  `0x140D86F00(actor, pos, cell, 0.875)` → `0x140D86F30` = (water height − pos.z) / height (`0x140D866A0`),
  water height from `0x1403B4D90` (cell `+0x40` bit 1 has water, `+0x60` water height; also scans loaded exterior
  cells). pos = `0x140D703B0`: the ragdoll's bounds centre (game units), else the actor position. If the centre is
  not deep enough, an all-hits ray up half the height re-tests at a hit point (water surface). No front/back choice
  in code: face-up vs face-down is picked inside the behavior graph *(not traced)*. Then the get-up action (default
  object `+0x360`, `0x140E86640`) and knockState 5. Also clears the face's dead flag (`SetDead(0,0)`).
- Graph event handlers (vtables `[dtor, handler]`): RemoveRagdoll `0x142D4C4D8`, RagdollStart `0x142D4C4F0`,
  GetUpStart `0x142D4C508` (→ `0x140F70850`: place controller at ragdoll, knockState 6), GetUpEnd `0x142D4C520`
  (→ `0x140F70AD0`: remove ragdoll, queue task 0xC), RagdollAndGetUp `0x142D4C538`.
- Task 0xC → end knockdown `0x140E743E0` (ID 1355205): remove ragdoll, knockState 0.
- Every get-up plays a "from the floor" animation.
- Behavior events seen on the graph: `WeaponRagdoll`, `RagdollInstant` (accepted but only keyframe a pose),
  `Action Knockdown` (rejected). Graph variable `bWantFullRagdoll` exists.
- Task queue: `TaskQueueInterface` singleton; `TaskUnpackFunc` ID 1546751 `0x140D60440` is a big switch on
  task type (10 knock explosion, 0xC end knock, 0xD start get-up, …).

---

## 6. Hits, staggers, death

- `Actor::DoHitMe(HitData*)` ID 881215 `0x140E01630`; only caller: `TaskUnpackFunc` at `+0x921`
  (`0x140D60D61`, actor in RBX, HitData in RDI, main thread). Hook there to see every hit before it applies.
- `HitData` (`0xE0`): `DamageImpactData` at 0 (members private: location `+0x00`, normal `+0x10`, projectile
  velocity `+0x20`, `bhkNPCollisionObject*` of the hit body `+0x30` → `GetBodyId`), aggressor `+0x40`,
  totalDamage `+0x94`, stagger `+0xAC`, flags `+0xC4` (kFatal …).
- Stagger: in DoHitMe, `stagger > 0` → `0x140DA6A40(actor, stagger, default object +0x218 (stagger action),
  hit)`, else the normal hit-reaction action (`DAT_1459DBF48`). Zeroing `HitData::stagger` removes the stagger
  for that hit only. INI `bPlayStaggers:Combat` exists *(not traced)*.
- `Actor::HandleHealthDamage` vfunc `0x110` `0x140E01460` is a death handler (runs after health ≤ 0), not the
  damage applier. The health modifier is `0x140DF7140` (`ActorValueOwner` slot 6 thunk `0x140DF7110`).
- `0x140D70010(actor, args)` = perform the default-object action at `DefaultObjectManager + 0x370` (objects start at
  `+0x20`, 8 bytes each → index 0x6A `kActionRagdollInstant`) via a TESActionData; KillImpl uses it for the instant
  death ragdoll. `Actor::PerformAction(BGSAction*, target)` with `GetDefaultObject<BGSAction>(...)` does the same.
- Actor bool flag `kAttackingDisabled` = `Actor + 0x43C` bit 20 (`0x100000`): several setters (`0x1408AD4B6`,
  `0x140FF4E3D`, `0x1411BC26E`, …), readers e.g. `0x1408908D6` (`mov eax,[rbx+0x43C]; shr eax,0x14; test al,1`).
- `kActionKnockdown` / `kActionRagdollInstant` on a living NPC (even after undoing our registration) did not set
  knockState: a prone animation, no ragdoll.
- `AIProcess::KnockExplosion(actor, location, magnitude)` ID 533106 `0x140E73F40` → `0x140D598B0`: queues task type
  10 {actor (ref-counted), x, y, z, magnitude} to the same task queue / unpacker as DoHitMe (`TaskUnpackFunc`). This is
  the explosion knockdown (real ragdoll + get-up). Used by DynamicRagdollSystem for burning falls and hit knockdowns.
  The push gives every ragdoll body the same speed (no falloff from the location: the whole rig moves as one block).
  Magnitude 0 still ragdolls the actor and it still gets up, just without the engine push; hit knockdowns use that
  and apply their own push falling off with distance from the bullet's impact point (`DamageImpactData` location).
- `Actor::KillImpl(attacker, damage, sendEvent, ragdollInstant)` vfunc `0x117` `0x140DB27B0` (vtable entry
  `0x142D3FC70`): with `ragdollInstant` → `0x140D70010` (instant ragdoll); without → performs the death action
  (default object `+0x300`, death animation). Death ragdoll bodies are dynamic on layer 32.

### Face on death (`BSFaceGenAnimationData`, layout in CommonLibF4)

- Getting it: `0x140E703C0(AIProcess*, Actor*)` ID 557396 → `middleHigh->faceAnimationData` (`+0x3C8`), else
  `GetFaceNodeSkinned()` (vfunc `0x408`) `->faceGenAnimationData` (`+0x170`).
- `SetDead(data, bool dead, bool clearSpeech)` `0x140668840` ID 110770. `dead=1` (only if not already dead):
  blinkTimer = 1 or 0, blinkState = `kPreDead` (5), `actorIsDead` (`+0x2DB`) = 1, dirty. `dead=0` (only if
  dead): `actorIsDead` = 0, blinkState = `kPreBlink` (0), dirty. Not called per frame. Callers:
  - `0x140E73F80` knockdown state machine: entering knockdown/unconscious → `SetDead(1,0)` (this closes the
    eyes when a death ragdoll goes down);
  - `0x140E60860` head/3D setup: `SetDead(IsDead || lifeState == 7, 1)`;
  - get-up / recovery `0x140E743E0`, `0x140E86810`, bleedout `0x140E73480` (lifeState 7): `SetDead(0,0)`;
  - console/script-style thunks `0x140E78BF0` (0,0), `0x140E78C30` (1,0), `0x140E78C70` (1, + face node
    flag `0x100`); `0x1400E6130` (1).
- `Update(data, dt, doBlink)` `0x1406689D0` ID 3139, called only from the face node update `0x140686040`
  (needs face node flags `+0x17C` bit `0x4`; always passes doBlink = 1). With `actorIsDead` set and blinkState
  ≠ 4: both eyelid morphs (`finalExpression[idx]`, indices in globals `0x1437333F8` / `0x143733410`, closed value
  `0x1437333E0`) are forced shut and blinkState = `kBlinkDead` (4). With `actorIsDead` set the
  speech/emotion/lip-sync blend into `finalExpression` is skipped (face frozen).
- Blink state machine `0x140668170`: 0 counts down then → 1 (closing) → 2 (opening) → 0; 3 → 0; 4 while dead
  ramps the lids shut, 4 while *not* dead → 2 (eyes reopen). Any other state (5) does nothing. Lid pose writer
  `0x1406683C0(data, amount)` ID 165948: amount 0 = rest (`lidsFollowEyes` `+0x2B0`), 1 = shut.
- So: `SetDead(0,0)` revives blinking/expressions on a dead actor; lids at rest + blinkState 5 with
  `actorIsDead` clear freezes the eyes open.
- Expression arrays (54 floats; speech `+0xF0`, emotion `+0x1C8`, final `+0x18`). Indices 0–49 are named in
  the exe's string table at `0x142CEB388` (alphabetical = index order): BrowSqueeze, JawFwd, JawOpen,
  LBrowOutUp, LCheekUp, LFrown, LJaw, LLipCornerIn, LLipCornerOut, LLwrLidDn, LLwrLidUp, LLwrLipDn, LLwrLipUp,
  LMidBrowDn, LMidBrowUp, LNoseUp, LOutBrowDn, LSmile, LUprLidDn (18 = left blink), LUprLidUp, LUprLipDn,
  LUprLipUp, LwrLipFunnel, LwrLipRollIn, LwrLipRollOut, Pucker, RBrowOutUp … RUprLipUp (26–44, RUprLidDn 41 =
  right blink), StickyLips, UprLipFunnel, UprLipRollIn, UprLipRollOut, Tongue. Blink indices are INI settings
  (values at `0x1437333F8` = 18, `0x143733410` = 41); closed lid value `0x1437333E0` = 1.0.
- Not dead: final = emotion (or max(speech, emotion) while lip-syncing), then each lid = blink value +
  emotion[lid]. The emotion layer is written by an emotion-instance pool (`0x145A5DC80`, 0x24-byte entries,
  8 handler types in the table at `0x143757EA0`, handle `+0x2A4`; `0x1409C15C0`). Writing emotionExpression
  directly shows up in the face when no instance is active (e.g. dead actors). No "pain" emotion exists; dialogue
  emotions are Neutral/Anger/Fear/Happy/Sad/Surprise/Puzzled/Disgusted.

---

## 7. Hooking notes (CommonLibF4)

- `REL::THook` only accepts thunks (`E8/E9/FF15/FF25`); most functions here have normal prologues.
- Used: vtable patches (`REL::THookVFT`: `bhkWorld` slot 0x28, `BShkbAnimationGraph` slot 19, `Actor` slot
  0x117) and 5-byte call-site patches (`REL::GetTrampoline().write_call<5>`: driveToPose, movement query,
  DoHitMe). Verify the `E8` target before patching.
- A vtable hook that must only affect one caller can filter on `_ReturnAddress()`.

Rigid-body control palette (track 13): DynamicRagdollSystem writes two entries while an NPC burns (entry 1 = soft
arms) and sets the per-bone map byte (after the `capacity × elementSize` data, 16-byte aligned) to 1 for the arm
ragdoll bones, assuming the map is indexed by ragdoll skeleton bone (capacity matched the ragdoll's bone count, 17/18).
*(Unverified beyond that; if burning arms stay stiff, check the map indexing.)*

