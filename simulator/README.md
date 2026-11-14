# RagdollLabSim

A standalone stand-in for Fallout 4's Havok ragdoll, built on [Jolt Physics](https://github.com/jrouwe/JoltPhysics)
and [raylib](https://www.raylib.com). It runs the plugin's puppet controller on the game's own human ragdoll, so you
can tune `[Puppet]` and the death mechanics in seconds instead of restarting the game. It's a separate xmake project
and shares no code with the plugin: its controllers are ports (see the end).

## Build and run

```
build.bat config     first time (downloads Jolt, raylib, nlohmann_json, toml11)
build.bat run        build and start
```

Or run `build\windows\x64\releasedbg\RagdollLabSim.exe [settings.toml] [ragdoll.json] [plugin.toml]` from this folder.
`--test` runs without a window: the puppet stands 5 s, walks 3 s, turns 2 s, walks back 2 s and stops 2 s, then is
shoved over forward, backward, left and right (limp for 1.5 s each time), knocked down (key 1), and switched to the
weak walker (key 4), which must stand still on its own and fall from an `F` push (`fPushSpeed`); each time it must get
up by itself without kicks or launches. Then it is shot in the head from four sides, and the clutching hand must reach
within 0.25 m of the head, also with both hands first pulled behind the buttocks (a hand behind the back used to snag on it).
Then it is shot in each upper arm from four sides (standing, and from the front while walking): the other hand must
clutch it (an arm can't clutch itself) and come within 0.25 m of the shoulder joint. Chest shots from four sides print,
with these, how long the free hand spends behind the back and how much it moves there (near 0 = held stiff). It prints gait
numbers (both feet up, highest foot, foot behind the pelvis, planted-foot slip), each get-up's stages and time, and
PASS or FAIL (exit code 0 means PASS). Each death (walk, stagger, stumble) is also shot in the chest from behind: it
must step, clutch the wound, not be launched and end up down; the walk must still be up at 1.5 s and take 4 steps.
`--head` runs only the head shots, `--arms` only the arm and chest shots.
`--stumble` (no pass/fail) shoots the puppet, standing and walking, in the chest, head and thigh from all four sides,
twice, then in the chest from the front, back and left at push scales 0.5, 1, 1.5, 2 and 3, and prints per shot and per
group, with steps read off the feet (lift-off to touch-down at least 5 cm away while the pelvis is above half height):
steps along, against or off to the side of the stagger line, mean / longest / first step length, when the first foot
lifted, how much of the pelvis's speed along the line was gone at each touchdown, how far planted feet slid
(shuffling), how far the pelvis travelled along the shot, when it dropped below half height and how high it was when
the stumble ended, how far the body turned while upright, frames with a stiff leg stuck out (foot in the air, knee
straight, far from the hip), frames with the legs tangled (a foot on the wrong side of the other, close to it), and
once the legs give way: how far apart the feet get along the line and frames of a split (a near-straight leg with its
foot over 0.5 m from under the hip while the pelvis is below 0.7 of standing height).
It also prints how it falls (pelvis height and trunk tilt when it goes limp, the longest time standing still upright or
low, the largest one-frame change in strength or support, steps taken leaning over 30 degrees, fall direction against
travel) and how hard the hit throws the hit part (its peak speed relative to the pelvis, and the trunk's bend, within
0.3 s).
`--frames` saves contact sheets (side on to the shot, a frame every 0.1 s for 3.6 s) of a few stumbles:
`front_chest.png`, `side_chest.png`, `back_chest.png`, `walking_front_chest.png`, `walking_side_thigh.png`,
`front_thigh.png`, `side_thigh.png`, and of the first 0.4 s of a hit every 0.033 s: `hit_chest.png`, `hit_head.png`,
`hit_shoulder.png`, `hit_thigh.png`, and facing the body (front on, 2.5 m away) a right upper arm and a chest shot from
the front: `facing_shoulder.png`, `facing_chest.png`, in the working folder (a hidden window renders them; it needs a display). Shots
from a key (`Q`/`W`/`E`) and `--frames` use push scale 1.5 (`kShotScale`: the plugin scales its push by damage, so a typical rifle
hit in the game is 1.2-2 times its default); `--test` and `--stumble` use 1. `--test`, `--stumble` and `--frames` seed
the per-death random rolls, so runs repeat exactly.

## Keys

Killing shot on the body part under the mouse (`src/Death.cpp`; then it goes limp, `R` or `3` to stand again):
- Left click: the plugin's roll (`Death::Roll`, as `StartPendingDeaths`): `fDeathWalkChance` (only headless if
  `bDeathWalkHeadlessOnly`, and the sim never dismembers, so then never), else `fDeathStaggerChance`, else
  `fDeathStumbleChance`; a head shot may then skip it (`fHeadshotSkipStumbleChance`), and the grip is rolled
  (`fDeathGripChance`). Nothing rolled: a plain ragdoll fall. The console prints the roll.
- `Q`, `W`, `E` always give that death, with the grip:
- `Q` death walk (`../src/Walk.cpp`): wanders drunkenly on its feet while its strength drains, then collapses.
- `W` death stagger (`../src/DeathStagger.cpp`): the walker staggers under the hit (`Puppet::Stagger`) and falls the
  way it was going; brace too.
- `E` death stumble (`../src/DeathStumble.cpp`): the whole-rig balance controller (tilt, height, capture-point steps,
  joint servo with knee buckle); brace too.

Arrow keys walk relative to the camera (`Shift` runs) · `1` knock down (shove along the camera, then it gets up) · `4` weak walker on/off · `3` puppet on/off (off = limp; back on, it gets up from wherever it
lies, and the top line shows the get-up stage) · `F` push the chest ·
`R` reset · `P` pause · `N` step one frame while paused · right mouse button orbits · mouse wheel zooms

Colours: orange = trunk held up by the controller, red = the foot currently stepping, dark red = the part that was shot, green = the hand clutching it, yellow dot = the target pelvis
position, blue rings = where each foot will land.

## Settings

Two files, both reloaded within half a second of a save. Each is looked for next to the exe first, then in the repo's
`res` folder (`../../../../../res` from the exe); the bottom lines of the window show whether each loaded.

`RagdollLab.toml` (written to `res` with defaults if it is in neither):
- `[Puppet]`: the walker (keys 3 and 4, and the walk and stagger deaths). The plugin's walker has these values as
  constants (`../src/Walker.h`).
- `[Sim]`: simulator only: time scale (slow motion), gravity, friction, damping, solver iterations, push speed.
- `[Stumble]`, `[Ragdoll]`: left over from an older plugin; unused.

`DynamicRagdollSystem.toml`, the plugin's own file (must be in one of the two places, or the sim exits with an error).
The death mechanics read these keys from it, with the plugin's ranges (`src/PluginSettings.h`), and the sidebar on the
right tunes them live:
- `[Reaction]` `fHitPush`, `fBodyPush`
- `[Death]` `fHeadshotSkipStumbleChance`, `fDeathGripChance`, `fDeathStumbleTime`, `fDeathPushScale`, `fDeathBrace`
- `[DeathStumble]` `fDeathStumbleChance`, `fDeathSupport`, `fDeathStiffness`, `fDeathUpright`, `fDeathKneeBend`, `fDeathCatchSpeed`/`Range`,
  `fDeathBraking`
- `[DeathStagger]` `fDeathStaggerChance`, `fDeathStaggerStep`, `Tip`, `Slump`, `HurtLeg`, `Flinch`, `GiveUpTilt`, `CatchSpeed`/`Range`, `Braking`
- `[DeathWalk]` `fDeathWalkChance`, `bDeathWalkHeadlessOnly`, `fDeathWalkTime`, `PushScale`, `ImpactSupport`, `ImpactTime`, `Speed`, `TurnRate`, `Weave`, `WeaveTime`,
  `Lurch`, `Drift`, `DriftGap`, `SharpTurnChance`

Ranges are the `Min`/`Max` pairs. The rest of the file (blacklists, twitches, crawl, explosions, crouching, ...) has
no effect in the sim, so it is not in the sidebar. The balance controller's fixed constants (`../src/Common.h`) are constants at the top of `src/Death.cpp`.

Sidebar (its own panel right of the 3D view): **Reset** puts every value back to the file's (as last loaded or
saved). Drag a slider to change a value (hold `Shift` for 10x finer), click a box to flip a true/false; the mouse wheel scrolls the list. A Min can't
go past its Max. `*` after the file name = unsaved changes. **Save** writes every value shown into the file in place:
only those value lines change (whole numbers get a `.0`, flags are written `true`/`false`), comments and every other key are kept, and a key the file
lacks is added under its section. Saving or editing the file elsewhere reloads it (unsaved slider changes are lost).

## What comes from the game

`human_ragdoll.json` was dumped from Fallout 4 1.10.163 with Cheat Engine (offsets in `../RE_NOTES.md`). It holds:
- for each of the 18 bodies: name, position, orientation, capsule shape and mass;
- for each of the 17 joints: the frame on each side, twist / cone / plane limits or hinge limits, and friction.

Mapping to Jolt:
- Havok ragdoll joints → `SixDOFConstraint` with pyramid swing, because Jolt's swing-twist joint can't do Havok's
  asymmetric plane limits.
- Hinges → `HingeConstraint`.
- Bodies of the same ragdoll don't collide with their parent or child body (FO4's filter does the same).

## What does not match the game

- Solver, contact softness, step rate: Jolt is not hknp. Expect gains to carry over roughly, not exactly.
- Damping and surface friction were never traced in the game. The `[Sim]` defaults are Jolt's own.
- Inertia comes from the capsules, not the game's values, and cones are pyramids.
- The floor is flat. In the game the puppet reads the ground height with a ray every frame.
- Left out of the deaths: weapon knockback trim, explosions and heavy-hit flings, dismemberment (so no headless
  walk), crouching, flailing, twitches and the crawl, a walk turning into a stumble when shot again, and the dying
  phase.
- The controllers are ports: `src/Puppet.cpp` of `../src/Walk.cpp` and `../src/DeathStagger.cpp`, `src/Death.cpp` of
  `../src/DeathStumble.cpp`, `../src/Walk.cpp` (`StepDeathWalk`) and `../src/Death.cpp` (grip, brace). When you change
  a controller, change both.
