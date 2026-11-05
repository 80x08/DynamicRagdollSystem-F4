# Fallout 4 AE — address map for DynamicRagdollSystem

OG 1.10.163 stays the reference and main target: `RE_NOTES.md` explains *what* everything is. This file only maps
the OG addresses the plugin uses to later runtimes. One section per runtime; "ID" = Address Library ID of that
runtime's bin (`BethesdaGhidraScripts-v1.2.3/addresslibrary/f4/version-*.bin`).

How to redo it for the next runtime is at the end.

---

## 1.11.240

Exe: `BethesdaGhidraScripts-v1.2.3/exes/f4/240/Fallout4.exe.unpacked.exe` (Steamless). Bin: `version-1-11-240-0.bin`.
Different compiler from OG: prologues, register choice and inlining differ, so raw OG byte signatures mostly fail.
Function IDs are all new (2.2M+ range); **vtable / RTTI IDs keep their OG numbers** (CommonLibF4's
`RE::VTABLE::*` IDs resolve correctly on AE).

### Address Library IDs

| OG ID | What | OG | AE ID | AE | Matched by |
|---|---|---|---|---|---|
| 137330 | `bhkWorld` add step listener | `0x141D72090` | 2277747 | `0x141877DE0` | called twice at the top of `bhkWorld::Update`, same call order as OG |
| 488591 | `hknpBSWorld` linear impulse | `0x141D6D180` | 2721630 | `0x141872940` | same slot in the hknpBSWorld wrapper block (CommonLib pairs neighbours `SetBody*Velocity` 2277670–72, `ActivateBody` 2277677); callee = applyLinearImpulse (242 vs 243 bytes, same mul count) |
| 222853 | `hknpBSWorld` angular impulse | `0x141D6D1B0` | 2721631 | `0x141872970` | next wrapper; callee = applyAngularImpulse (364 vs 365 bytes) |
| 357289 | `bhkUtilFunctions::SetMotionType` | `0x141D70E90` | 2277723 | `0x1418769C0` | CommonLib `IDs.h` + byte signature |
| 715635 | set collision layer on tree | `0x141D8DA30` | 2278105 | `0x14188C850` | unique 96-byte masked signature |
| 469495 | `bhkCollisionFilter*` global | `0x1458E0AD8` | 4797438 | `0x1430F83C0` | only data ref shared by `bhkWorld::Init` (slot 0x29) and filter slot 4, in both builds |
| 147831 | filter pair check (doc only) | `0x141D88E60` | 2278061 | `0x141887D50` | callee of filter slot 4; same `+0x1A0` / `+0x3A0` reads |
| 1346879 | sync anim manager singleton | `0x1459D7608` | 4798289 | `0x1431EE438` | CommonLib `IDs.h`; loaded as `rcx` for a manager method (`0x140E01790`) |
| 1291193 | sync instance "is actor in" | `0x14080CE90` | 2214404 | `0x14080CD10` | instruction-for-instruction match (0x150 stride, `+0x108` handle, `+0x142` flag) |
| 1546751 | `TaskUnpackFunc` | `0x140D60440` | 2229323 | `0x140C4D320` | CommonLib; only caller of DoHitMe |
| 881215 | `Actor::DoHitMe` | `0x140E01630` | 2231148 | `0x140CD2CA0` | CommonLib; stagger read at `+0xAC` |
| 737625 | actor movement update (`Move`) | `0x140D8CD80` | 2229934 | `0x140C76FE0` | CommonLib; aligned call list |
| 453608 | slot-19 visitor | `0x140DD4D70` | — | — | **inlined into `Move`** (see call sites) |
| 1348502 | graph pre-physics update | `0x14161C520` | 2256836 | `0x14131BE40` | signature + identical 10-call list |
| 993992 | `hkbRagdollDriver::driveToPose` | `0x1416FDE80` | 2258034 | `0x141393F50` | 4th call of pre-physics update in both; same `+0x70` / `+0xB7` early-outs |
| 1414229 | graph post-physics update | `0x14161C6F0` | 2256837 | `0x14131C020` | signature + identical 17-call list |
| 560275 | `hkbRagdollDriver::postPhysics` | `0x1416FF3A0` | 2258035 | `0x1413956D0` | 7th call of post-physics update in both |
| 1273937 | graph per-frame update | `0x14161D9A0` | 2256850 | `0x14131D420` | 16th call of post-physics update in both |
| 683498 | AddRagdollToWorld visitor | `0x140D643B0` | 2230588 | `0x140CB6750` | **signature changed**, see below |
| 258136 | register ragdoll visitor (slot 3) | `0x140D64280` | 2230583 | `0x140CB6160` | **signature changed** |
| 385024 | remove ragdoll visitor (slot 4) | `0x140DD5310` | 2230587 | `0x140CB6620` | **signature changed** |
| 69396 | per-graph helper add (slot 5) | `0x140821160` | 2214611 | `0x14081BC50` | masked signature; `(args*, graph**)` unchanged |
| 1039942 | per-graph helper register (slot 3) | `0x140821110` | 2214610 | `0x14081BC00` | called by 2230583, does `call [rax+0x18]` |
| 428366 | per-graph helper remove (slot 4) | `0x1408210C0` | 2214609 | `0x14081BBB0` | called by 2230587, does `call [rax+0x20]` |

### Call sites

| Hook | OG | AE | Instruction |
|---|---|---|---|
| DoHitMe | 1546751 + `0x921` (`0x140D60D61`) | 2229323 + `0x8F7` (`0x140C4DC17`) | `E8` → DoHitMe |
| driveToPose | 1348502 + `0x12F` (`0x14161C64F`) | 2256836 + `0x144` (`0x14131BF84`) | `E8` → driveToPose |
| postPhysics | 1414229 + `0x1EA` (`0x14161C8DA`) | 2256837 + `0x1DC` (`0x14131C1FC`) | `E8` → postPhysics |
| graph update "ragdoll active" | `0x14161D9F4` (1273937 + `0x54`) | 2256850 + `0x69` (`0x14131D489`), return `0x14131D48F` | `FF 90 98 00 00 00` (`call [rax+0x98]`) |
| movement "ragdoll active" | 737625 + `0x3B8` (`E8` → visitor 453608) | 2229934 + `0x4ED` (`0x140C774CD`), return + `0x4F3` (`0x140C774D3`) | `FF 90 98 00 00 00`, inside the inlined visitor loop |

### Vtables (same IDs as OG)

| Class | ID | OG | AE |
|---|---|---|---|
| `BShkbAnimationGraph` | 742655 | `0x142DBEE58` | `0x14262EC68` |
| `bhkWorld` | 356317 | `0x142E3D568` | `0x1426AC920` |
| `bhkWorldM` | 1488354 | `0x142E433C8` | `0x1426B2638` |
| `Actor` | 1455516 | `0x142D3F3B8` | `0x1425683B0` |
| `hkbnpRagdollInterface` | 1488961 | `0x142DC2B88` | `0x142632508` |
| `bhkCollisionFilter` | 599770 | `0x142E3ECA8` | `0x1426ADF50` |
| `hkpRagdollConstraintData` | 760737 | `0x142DD69E8` | `0x142644DC8` |
| `hkpLimitedHingeConstraintData` | 1540231 | `0x142DD5548` | `0x142643930` |
| `bhkIWorldStepListener` | 1274460 | `0x142DBEDF8` | `0x14262E758` |
| `hknpBSCustomCollisionFilter` | 1200668 | `0x142C4CF58` | `0x14246B2F8` |

Slot numbers are unchanged. Actor base-class offsets (`+0x20 … +0x168`) are identical.

| Slot | OG | AE |
|---|---|---|
| graph 1 has ragdoll (`+0x210`) | `0x141626C20` | `0x141326FA0` (2256886) |
| graph 3 register | `0x141626C80` | `0x141326FF0` (2256888) |
| graph 4 remove | `0x141627080` | `0x141327500` (2256889) |
| graph 5 `graph+0x3B8 = world` | `0x141627180` | `0x1413275E0` (2256890) |
| graph 7 reset | `0x1416271B0` | `0x141327610` (2256892) |
| graph 19 ragdoll active | `0x141627BD0` | `0x141328130` (2256904) |
| `bhkWorld`/`bhkWorldM` 0x28 Update (not overridden in M) | `0x141D6EC00` | `0x141874440` (2277710) |
| `bhkWorld` 0x29 Init | `0x141D712C0` | `0x141876DF0` (2277731) |
| `Actor` 0x117 KillImpl | `0x140DB27B0` | `0x140CA00C0` (2230339) |
| interface `+0x20` IsAdded / `+0x78` remove / `+0xA0` add | `0x1417363F0` / `0x141736E30` / `0x1417370C0` | `0x1413BA070` / `0x1413BAA90` / `0x1413BACF0` |
| filter slot 4 | `0x141D890B0` | `0x141887F80` (2278063) |

### Field offsets checked on AE (all unchanged)

Every struct field the plugin reads or writes was compared in a function matched in both builds.

| Struct | Fields | Checked in |
|---|---|---|
| Graph | `+0x208` driver, `+0x210` interface, `+0x3B8` world | graph slots 1/2/5/19 |
| Ragdoll driver | `+0x70` interface, `+0xB7` enable | driveToPose early-outs (68 shared offsets, rest is loop noise) |
| `hkbnpRagdollInterface` / `hknpRagdoll` / skeleton | interface `+0x18`; ragdoll `+0x18` world, `+0x20` bodies, `+0x30` constraints, `+0x40` skeleton, `+0x48` map; skeleton `+0x18` / `+0x28` | all 26 interface vfuncs read identical offset sets |
| Generator output | count `+4`, track headers `+0x10 + i·0x10` | driveToPose / postPhysics (postPhysics identical) |
| `hknpWorld` | bodies `+0x20` (stride `0x90`), motions `+0xE0` (stride `0x80`), constraint buffer `+0x128` / count `+0x130` (stride `0x38`), debug lock `+0x690` | applyLinear/AngularImpulse, interface add |
| `hknpBSWorld` | world lock `+0x6D8` | `bhkWorld::Update` |
| `hknpBody` | flags `+0x40`, filter info `+0x44`, motionId `+0x68` | applyLinearImpulse, filter slot 4 |
| `hknpMotion` | orientation `+0x10`, packed inertia `+0x20`, linear vel `+0x40`, angular vel `+0x50` | applyLinear/AngularImpulse |
| `bhkCollisionFilter` | layer masks `+0x1A0` (u64 ×64), per-bone `+0x20 + bone·4`, `+0x3A0` | pair check |
| Constraint data | ragdoll `0x1A0` bytes, hinge `0x130`; `hkReferencedObject` mem size `+0xA` | deleting dtors |
| `hkMemoryRouter` | `heap` `+0x58` | constraint deleting dtor |
| `bhkPickData` | ctor layout, start/end `+0x20` / `+0x30`, `customCollideLayers` `+0xC8` | ctor, `SetStartEnd`, `bhkWorld::Pick` (AE `0x141876060`, ID 2277718) |
| TESObjectREFR | `+0xB8` parentCell, `+0xC0` data (angle), `+0xD0` location | REFR vfuncs (13/13 slots read `+0xB8` in both) |
| Actor | `+0x48` graph holder, `+0x110` MagicTarget, `+0xC8` heading (`data` `+0xC0`), `+0x300` currentProcess, `+0x418` race, `+0x43C` boolFlags | Actor vtables (base offsets identical), heading/race vfuncs (10 slots read `+0x418`) |
| ActorState (`+0x128`) | `+0x130` lifeState bits 17–20 (`IsDead` masks `0x1E0000`), knockState 21–24 (knock update masks `0xFE1FFFFF`), meleeAttackState 25–27 (`0xE000000` / `shr 0x19`); `+0x134` interactingState bits 18–19 (37× `and 0xC0000`, no neighbouring masks) | IsDead, knock update, byte patterns |
| boolFlags bits | kIsInKillMove 14, kEssential 18, kAttackingDisabled 20 | `or`/`and` and `shr` counts on `+0x43C` match OG |
| AIProcess / middleHigh | `+0x08` middleHigh; `+0x238` activeEffects, `+0x258` animationGraphManager, `+0x3A8` occupiedFurniture, `+0x430` bleedoutTimer | `GetActiveEffectList`, graph-manager getter, `GetOccupiedFurniture`, knock update |
| Graph manager | `+0x40` inline flag, `+0x48` graphs, `+0x50` count, `+0xC8` lock | visitor loops |
| Sync manager / instance | `+0x18` list, `+0x28` count; instance `0x150` bytes, handle `+0x108`, flag `+0x142` | `isIn`, manager loop |
| `HitData` | all 16 DoHitMe offsets `+0x40…+0x116` identical (stagger `+0xAC`, flags `+0xC4`) | DoHitMe |

*(Inferred)* `DamageImpactData` velocity `+0x20` and collision object `+0x30`: not read in DoHitMe, but every HitData
field after the impact block is at its OG offset, so the block kept its size.

Knock update (OG `0x140E73F80`) = AE `0x140D359D0` (ID 2232385, right after `KnockExplosion` 2232384 as in OG).

### Default objects

The `DEFAULT_OBJECT` table (ID 838886, same in both bins) has 397 entries on AE vs 394 on OG. AE inserts
"Installed Content" at index 230. Every action the plugin uses is below that and unchanged: `kActionDeath` 0x5B,
`kActionKnockdown` 0x67, `kActionGetUp` 0x68, `kActionRagdollInstant` 0x6A (manager `+0x370`).

### CommonLibF4 functions

Of 1,689 IDs in CommonLibF4's `RE/IDs.h`, only 16 are OG-only (Pipboy constraints, console/script tables, two
INI floats, `hkLifoAllocator::Init/Quit`); the plugin uses none of them. Everything it calls (`bhkPickData`,
`TESObjectCELL::Pick`, `AccessMotion`, `GetAVObjectFromBodyID`, `GetBodyId`, `PerformAction`, `KnockExplosion`,
`ModActorValue`, …) has an AE ID or is virtual.

### Differences the code must handle

All handled in code (`InstallMovementHook` / `Hook_IsRagdollActive`, `GraphVisitorTarget`, `ResolveImpulseFunction`,
`InstallGraphUpdateHook`).

1. **Movement query has no call to patch.** The slot-19 visitor is inlined into `Move`. Use the existing slot-19
   vtable hook and also answer "no" when `_ReturnAddress()` is 2229934 + `0x4F3`, as the graph-update hook already
   does.
2. **Ragdoll visitors take the holder, not the manager.** AE 2230588 / 2230583 / 2230587 are
   `bool(IAnimationGraphManagerHolder*, args*)`. They call holder vfunc `+0x20` (get manager), lock `+0xC8`, and loop
   over the graphs. Pass `static_cast<IAnimationGraphManagerHolder*>(actor)` (Actor `+0x48`). The engine does this too:
   `0x140CB2C50` calls 2230588 with the holder in `rcx`. Alternative for every runtime: loop over the manager's graphs
   and call graph slots 5 / 3 / 4 directly.
3. **Impulse fingerprints differ** (40 bytes, tail-jump to activate):
   - linear `48 89 5C 24 08 57 48 83 EC 20 8B DA 48 8B F9 E8 4C CD 9E FF 8B D3 48 8B CF 48 8B 5C 24 30 48 83 C4 20 5F E9 C8 07 00 00`
   - angular `48 89 5C 24 08 57 48 83 EC 20 8B DA 48 8B F9 E8 1C CE 9E FF 8B D3 48 8B CF 48 8B 5C 24 30 48 83 C4 20 5F E9 98 07 00 00`

   `ResolveImpulseFunction` also refuses any runtime other than 1.10.163.
4. Raw offsets in `InstallGraphUpdateHook` (`0x161D9F4`, `0x1627BD0`) need per-runtime values: AE `0x131D489` and
   slot 19 `0x1328130`, or derive them from IDs 2256850 + `0x69` and the vtable slot.

---

## Redoing this for a new runtime

Scripts used for 1.11.240 (Python + capstone, no Ghidra analysis needed) work on the unpacked exe and both bins:
- **Vtables:** find by RTTI name (`.?AVName@@` → type descriptor → complete object locator → vtable), then read
  slots. Check that the vtable ID still resolves.
- **Function bounds:** `.pdata`, following chained unwind entries (flag 4); leaf functions have no entry.
- **Matching:** in order of reliability: CommonLib `IDs.h` pairs → vtable slots → aligned call lists of an
  already-matched parent (callers/callees by position) → shared RIP-relative data refs between two matched functions
  → masked byte signatures (`E8`/`E9`/`Jcc` rel32 and RIP disp32 wildcarded) → shape match (same instruction
  sequence).
- Then read the new ID from the bin (offset → ID) and check every call site's bytes.
