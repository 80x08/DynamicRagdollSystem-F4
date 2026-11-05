#pragma once

// Shared types, settings, globals and function declarations. Implementations by area:
// Settings.cpp (TOML), Physics.cpp (bodies, joints, impulses), Reaction.cpp (non-lethal hit reaction),
// Death.cpp (death stumble, dying phase, ground rays, grip/brace), Hooks.cpp (world update).

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <intrin.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <string>
#include <initializer_list>
#include <limits>
#include <mutex>
#include <random>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace DynamicRagdollSystem
{
	using Clock = std::chrono::steady_clock;

	struct Quat
	{
		float x{}, y{}, z{}, w{ 1.0f };
	};

	struct Vec3
	{
		float x{}, y{}, z{};
	};

	inline float NowSeconds()
	{
		static const auto s_start = Clock::now();
		return std::chrono::duration<float>(Clock::now() - s_start).count();
	}

	struct BodyRef
	{
		std::int32_t   index{ -1 };
		RE::hknpBodyId id{};
		RE::hknpBodyId parentId{};
		std::int32_t   bone{ -1 };
		std::int32_t   parentIndex{ -1 };
		std::int32_t   constraintId{ -1 };
		bool           pelvis{ false };
		bool           spine{ false };
		bool           leg{ false };
		std::int32_t   priority{ 99 };
		bool           foot{ false };
		char           side{ 0 };  // 'L' / 'R' from the bone name, 0 if unknown
		bool           hand{ false };
		bool           arm{ false };  // upper arm, forearm or hand
		bool           knee{ false }; // calf body: its joint to the thigh is the knee
		bool           head{ false }; // head or neck body
	};

	struct Stumble
	{
		RE::TESFormID        formID{};
		float                start{};
		float                end{};
		float                nextScan{};
		std::vector<BodyRef> bodies;  // filled by ScanBodies
		std::vector<std::uint32_t> detached;  // body ids cut off by dismemberment (and everything below them)
		bool Detached(std::uint32_t a_id) const { return std::find(detached.begin(), detached.end(), a_id) != detached.end(); }
		std::vector<BodyRef> joints;
		Vec3                 hitDir{};  // unit direction of the hit
		std::uint32_t        hitBody{ 0x7FFFFFFF };
		float                pushScale{ 1.0f };  // HitPushScale of the hit
		bool                 pushed{ false };
		// diagnostics: actor (game units) vs trunk body (physics units) movement since the hit
		RE::NiPoint3         actorStart{};
		Vec3                 bodyStart{};
		RE::NiPoint3         lastActorPos{};
		float                trunkYawStart{ 0.0f };
		bool                 tracked{ false };
		std::uint32_t        layerResets{ 0 };  // times something put the bodies back on a CC-colliding layer
		float                nextTrace{ 0.0f };
		// On fire (living NPC): the reaction is kept going while the fire lasts, with flailing arms and shoves.
		bool                 burning{ false };
		float                burnUntil{ 0.0f };   // fire last seen + a little grace
		float                nextShove{ 0.0f };
		bool                 burnLogged{ false };
		float                burnStart{ 0.0f };    // when this burn began (0 = not burning)
		float                burnFallTime{ 0.0f }; // seconds until the burning NPC loses its balance and falls
		float                burnProgress{ 0.0f }; // 0..1 towards the fall
		bool                 burnFell{ false };    // knockdown triggered
		bool                 attackLock{ false };  // we set kAttackingDisabled (and must clear it)
		bool                 panicLock{ false };   // we applied the temporary Confidence drop (and must undo it)
		// Flail rhythm per joint (death flail and burning arms).
		std::vector<Vec3>    flailAxes;           // per joint: unit axis
		std::vector<std::pair<float, float>> flailRhythm;  // per joint: angular frequency, phase
	};

	using Mat3 = std::array<float, 9>;

	struct BodyTarget
	{
		std::uint32_t id{};
		Mat3          rot{};
		Vec3          pos{};
	};

	// Death stumble: the v0.1.x balance controller (pose hold, whole-rig balance, stepping, joint servo)
	// driving the engine's death ragdoll for a few seconds, then fading out so the body collapses.
	struct Death : Stumble
	{
		std::vector<BodyTarget> targets;
		// Rolled once per death from the settings chances/ranges.
		bool                    stumble{ true };
		float                   flail{ 0.0f };     // rolled flail strength
		float                   explosionPush{ 0.0f };  // blast velocity for a flung explosion death, 0 = none
		// Ground under the body from ray casts (UpdateGround, before each StepDeath). Heights are used as shifts
		// relative to the ground at the moment of death, so a missing ray falls back to the flat captured ground.
		bool                    groundCaptured{ false };
		float                   groundAtDeath{ 0.0f };   // ground under the feet when the pose was captured
		std::uint32_t           groundFootId[2]{};
		float                   groundFoot[2]{};         // ground under each foot (or ray end on a miss)
		int                     groundFeet{ 0 };         // feet cast this frame (1 after a leg was dismembered)
		float                   groundTrunk{ 0.0f };     // ground under the pelvis/lowest trunk body
		bool                    groundValid{ false };    // this frame's rays succeeded
		bool                    braceLogged{ false };
		float                   braceAmount{ 0.0f };   // this frame's brace strength, for the log
		float                   braceOff{ -1.0f };     // mean hand distance from its brace target, for the log
		float                   standHeight{ 0.0f };   // COM height above the ground on the first stumble frame
		std::string             weapon;                // killing weapon, for the log
		float                   damage{ 0.0f };        // killing hit damage
		bool                    flung{ false };        // explosion or heavy hit: no balance help, full knockback
		// Burning NPC knocked down (alive): no stumble/grip, only the dying-phase twitches and rolls while it
		// lies there burning. Held down until the fire is out (or kBurnDownMax), then the engine get-up runs.
		bool                    burnDown{ false };
		bool                    burnedToDeath{ false }; // died on fire: own twitch/roll strengths
		bool                    seenDown{ false };     // the engine reported the knockdown (knockState != 0)
		bool                    paralysisLock{ false }; // we applied the temporary Paralysis that keeps it down while burning
		std::uint32_t           lastKnock{ 0 };        // knock state last frame (log on change)
		float                   fireOutSince{ 0.0f };  // burn-down: when the fire effect was last seen gone (0 = burning)
		bool                    fallFallback{ false }; // kActionKnockdown did not take: tried kActionRagdollInstant
		Vec3                    moveVel{};         // actor velocity just before death, physics units/s
		float                   knockPeak{ 0.0f }; // largest extra speed (knockback) seen in the knock window
		bool                    knockLogged{ false };
		bool                    flailLogged{ false };
		bool                    grip{ true };
		float                   support{ 0.0f };
		float                   upright{ 0.0f };
		float                   braking{ 0.0f };
		float                   dyingTime{ 0.0f };
		float                   twitch{ 0.0f };  // 0 when the twitch chance failed
		float                   roll{ 0.0f };
		std::vector<std::pair<std::uint32_t, Vec3>> kneeAxes;  // calf body id -> flexion axis in the thigh frame
		bool                    kneeAxesDone{ false };
		Vec3                    feetCaptured{};
		bool                    hasFeet{ false };
		float                   lastStep{ 0.0f };
		bool                    writerGateLogged{ false };
		std::uint32_t           balanceSteps{ 0 };
		bool                    hitApplied{ false };
		std::uint32_t           stepFoot{ 0 };  // body id of the swinging foot, 0 when not stepping
		char                    stepSide{ 0 };
		char                    lastStepSide{ 0 };  // feet alternate: the next step uses the other foot
		float                   stepTime{ 0.3f };   // this step's duration (shorter when moving fast)
		Vec3                    stepFrom{};
		Vec3                    stepTo{};
		float                   stepStart{ 0.0f };
		float                   stepGround{ 0.0f };      // ground under stepTo (ray in UpdateGround, once per step)
		bool                    stepGroundTried{ false };
		bool                    stepGroundHit{ false };
		float                   nextStep{ 0.0f };
		std::uint32_t           steps{ 0 };
		std::uint32_t           gripHand{ 0 };   // body id of the hand clutching the wound, 0 = none chosen yet
		bool                    gripChosen{ false };
		float                   nextTwitch{ 0.0f };  // dying phase (after the stumble)
		std::int32_t            twitchJoint{ -1 };   // joint of the current twitch burst
		Vec3                    twitchAxis{};
		float                   twitchEnd{ 0.0f };
		std::uint32_t           twitchCount{ 0 };    // diagnostics: bursts started / impulses accepted
		std::uint32_t           twitchHits{ 0 };
		float                   nextRoll{ 0.0f };
		float                   rollEnd{ 0.0f };
		float                   twitchScale{ 1.0f }; // current twitch scaled by limb weight
		float                   rollSign{ 1.0f };    // direction of the current roll burst
		std::uint32_t           rollCount{ 0 };
		bool                    dyingLogged{ false };
		float                   downTime{ 0.0f };    // when the body was first lying (dying clock), 0 = not yet
		bool                    rollApplied{ false };
		float                   dyingDamage{ 0.0f }; // damage taken during the dying phase (ends it at kDyingEndDamage)
		bool                    dyingStepped{ false };  // StepDying has run once
		bool                    headOffBefore{ false }; // head already dismembered when StepDying started
		bool                    headCut{ false };       // head dismembered during StepDying: ends the dying phase
		bool                    crawlRolled{ false };   // fCrawlChance hit; crawls if it lies face down when the dying phase starts
		bool                    crawling{ false };      // dying phase is a crawl instead of twitches/rolls
		bool                    crawlDecided{ false };  // the face-down check is done (crawling or not)
		bool                    crawlRayValid{ false }; // CastCrawlRay result for this frame:
		Vec3                    crawlFront{};           // ... the body's front direction
		float                   crawlRayDist{ 0.0f };   // ... distance from the stomach to the ground along it (1e9 = no hit)
		std::uint32_t           crawlTurns{ 0 };        // diagnostics: rolls toward face down
		float                   crawlStart{ 0.0f };     // crawl motion starts (decision, or fCrawlDelay after down if later)
		float                   crawlTime{ 0.0f };      // ... and lasts this long (fCrawlTime)
		std::uint32_t           crawlLimbTries{ 0 };    // diagnostics since the last pull log: leg/head joint pushes tried ...
		std::uint32_t           crawlLimbHits{ 0 };     // ... and accepted
		bool                    headShotKill{ false };  // killing hit was to the head: no crawl (the game may take the head off later)
		bool                    crawlPlanted[2]{};      // per arm (L, R): hand planted for the current pull
		Vec3                    crawlPlant[2]{};        // ... where it was planted
		bool                    crawlPulling[2]{};      // ... planted ahead of the chest: this pull holds the hand and nudges
		float                   crawlBest[2]{};         // ... closest the hand got to its reach target this reach (0 = new reach)
		float                   crawlBestTime[2]{};     // ... when
		bool                    crawlStalled[2]{};      // ... stopped getting closer (joint limit): arm limp for the rest of the reach
		std::uint32_t           crawlPulls{ 0 };        // diagnostics: arm pulls started
	};

	struct LastHit
	{
		bool          explosion{ false };
		std::string   weapon;  // weapon name, for the log
		float         damage{ 0.0f };
		Vec3          dir{};
		std::uint32_t hitBody{ 0x7FFFFFFF };
		float         time{ -100.0f };
		std::uint32_t flags{ 0 };     // HitData flags after the engine applied the hit (diagnostic)
		std::int32_t  limb{ -1 };     // HitData damageLimb (diagnostic)
		float         pushScale{ 1.0f };  // HitPushScale (final damage and flags)
	};

	struct Pending
	{
		RE::TESFormID formID{};
		Vec3          dir{};                      // unit direction of the hit (projectile travel)
		std::uint32_t hitBody{ 0x7FFFFFFF };      // hknpBodyId of the body part that was hit, if known
		float         pushScale{ 1.0f };          // HitPushScale of the hit
		bool          burn{ false };              // started because the NPC is on fire
	};

	// hkbRigidBodyRagdollControlData (0x40 bytes). Defaults are the values the engine itself writes into
	// generator-output track 13 during a get-up (RE_NOTES, v0.1.32 probe).
	struct RigidBodyControl
	{
		float hierarchyGain{ 0.17f };
		float velocityDamping{ 0.0f };
		float accelerationGain{ 1.0f };
		float velocityGain{ 0.6f };
		float positionGain{ 0.1f };
		float positionMaxLinearVelocity{ 1.4f };
		float positionMaxAngularVelocity{ 1.8f };
		float snapGain{ 0.25f };
		float snapMaxLinearVelocity{ 0.3f };
		float snapMaxAngularVelocity{ 0.3f };
		float snapMaxLinearDistance{ 0.03f };
		float snapMaxAngularDistance{ 0.1f };
		float durationToBlend{ 0.5f };
		float pad[3]{};
	};
	static_assert(sizeof(RigidBodyControl) == 0x40);

	constexpr float kScanInterval = 0.1f;
	constexpr std::uint32_t kKnockGetUp = 6;  // ActorState::knockState: getting up (ragdoll blends back to animation)
	constexpr std::uint32_t kKnockDown = 7;   // ActorState::knockState: legless crawl (feral ghoul lost a leg): animation, no ragdoll
	constexpr float kReferenceRate = 30.0f;  // updates/s the controller gains are tuned for

	// Tunables, overridden by Data\F4SE\Plugins\DynamicRagdollSystem.toml (see LoadSettings).
	inline float kMinDamage = 5.0f;         // minimum damage of a single hit that triggers a reaction
	inline bool  kReactHumanoidOnly = false;     // hit reactions only on humanoid races (creature skeletons sink: no foot bones by name)
	inline float kCooldown = 0.1f;          // hits on the same NPC closer than this are ignored (shotgun pellets)
	inline float kReactionTime = 1.5f;      // how long the ragdoll is physical after a hit
	inline float kHitSoftness = 0.1f;       // controller gains right after the hit, as a fraction of the engine defaults
	inline float kRecoverTime = 0.8f;       // gains ramp from kHitSoftness back to 1 over this time
	inline float kBlendOut = 0.3f;          // ragdoll weight fades to 0 over the end of the reaction
	inline float kHitPush = 8.0f;           // velocity kick on the body part that was hit, physics units/s
	inline float kBodyPush = 5.5f;          // kick on bodies one joint from the hit body (halved per further joint), physics units/s
	inline float kPushDamageRef = 25.0f;    // damage that gives fHitPush/fBodyPush as set (0 = push ignores damage)
	inline float kPushScaleMin = 0.5f;      // damage scale clamp
	inline float kPushScaleMax = 2.0f;
	inline float kPushPowerAttackMult = 1.5f;  // extra scale for melee power attacks
	inline float kPushBlockedMult = 0.6f;      // extra scale for blocked hits
	constexpr std::int32_t kPushRings = 3;     // joints from the hit body that still get pushed
	constexpr float kFloorBodyHeight = 0.36f;  // body centre this far above the lowest one = floor level (~25 game units): no downward push
	constexpr float kFootBodyHeight = 0.1f;    // ... this far (~7 game units) = foot/paw: drive at full strength during a reaction
	inline float kMaxCatchUpSpeed = 7.0f;  // ragdoll may close the gap to the animated pose this fast, physics units/s
	inline float kAttackSoftness = 0.1f;     // stiffness floor while the NPC is in a melee attack (swings stay intact)
	inline float kAttackCatchUpSpeed = 9.0f;  // catch-up speed while in a melee attack (fast swings outrun the normal one)
	// [Burning]: living humanoid NPCs on fire flail and stagger.
	inline bool  kBurnReaction = true;
	inline float kBurnStiffness = 0.3f;      // body stiffness while burning (fraction of normal)
	inline float kBurnArmStiffness = 0.5f;   // arm stiffness while burning, so they can flail
	inline float kBurnFlail = 1.4f;          // arm flail strength while burning
	inline float kBurnShove = 2.5f;          // random stagger shove on the upper body, physics units/s
	inline float kBurnShoveMin = 0.5f;       // seconds between shoves ...
	inline float kBurnShoveMax = 1.2f;       // ... picked at random in this range
	inline float kBurnFallMin = 2.0f;        // seconds of burning before the NPC loses its balance and falls ...
	inline float kBurnFallMax = 4.0f;        // ... picked per burn (both 0 = never falls)
	inline float kBurnFallForce = 3.0f;      // knock force of the fall (engine explosion knockdown magnitude)
	inline float kBurnOutGrace = 1.0f;     // seconds without a fire effect before a burning NPC on the ground may get up
	inline float kBurnDownMax = 60.0f;     // safety cap on lying there burning (fire hazards can keep it lit)
	inline float kBurnDownRoll = 1.2f;     // roll strength of a burning NPC on the ground
	inline float kBurnRollGapMin = 0.4f;   // seconds between its rolls ...
	inline float kBurnRollGapMax = 0.8f;   // ... picked at random in this range
	inline float kBurnDeathTwitchMin = 2.6f;  // NPCs that die burning: twitch strength range ...
	inline float kBurnDeathTwitchMax = 3.0f;
	inline float kBurnDeathRollMin = 1.3f;    // ... and roll strength range (instead of the normal [Death] ones)
	inline float kBurnDeathRollMax = 1.8f;
	inline float kBurnDeathTimeMin = 6.0f;    // ... and how long it writhes (dying phase seconds)
	inline float kBurnDeathTimeMax = 10.0f;
	inline bool  kDisableDeathAnimation = true;  // NPC deaths go straight to ragdoll (KillImpl ragdollInstant)
	// Per-death random range: each death rolls a value in [min, max].
	struct Range
	{
		float min;
		float max;
	};
	inline float kDeathStumbleChance = 1.0f;     // chance a death uses the balance/stepping controller
	inline float kDeathGripChance = 0.75f;        // chance a hand clutches the body part the killing shot hit
	inline float kTwitchChance = 0.8f;           // chance the body twitches and rolls after it falls

	inline Range kDeathStumbleTime{ 2.3f, 2.6f }; // seconds the dead body tries to stay up before collapsing
	inline Range kDeathSupport{ 0.6f, 0.8f };     // fraction of gravity the controller carries at the moment of death
	inline float kDeathStiffness = 0.2f;         // scales joint and upright stiffness of the dying body
	inline float kDeathPushScale = 0.4f;         // killing-hit push as a fraction of fHitPush/fBodyPush
	inline Range kDeathUpright{ 0.4f, 0.6f };     // upright torque of the dying body: low lets it tip, so it has to step
	inline float kCatchSpeed = 4.0f;             // physics units/s: below this the dying body gets full help (running ≈ 4–5)
	inline float kCatchRange = 6.0f;             // ... fading to no help at kCatchSpeed + kCatchRange
	inline Range kDyingTime{ 3.0f, 15.0f };        // after the stumble: seconds of clutching/twitching before fully limp
	inline Range kCrawlTime{ 10.0f, 20.0f };        // seconds a crawl lasts from its start (after fCrawlDelay), fading out
	inline Range kTwitchStrength{ 1.0f, 2.5f };   // scales the dying twitches
	inline Range kRollStrength{ 0.5f, 1.5f };     // scales the dying rolls
	inline float kDeathKneeBend = 80.0f;         // degrees the knees fold by the end of the stumble
	inline Range kFlailStrength{ 2.0f, 3.0f };     // limb flailing while a fast/airborne body flies, 0 = limp
	inline float kDeathBrace = 2.5f;              // free hands reach for the ground when the body tips over (0 = off)
	inline float kDeathFlingDamage = 100.0f;      // killing hits with at least this damage fling the body (0 = off)
	inline float kDyingEndDamage = 20.0f;         // damage to the corpse during the dying phase that ends it (0 = off)
	inline bool  kDyingEndOnHeadLoss = true;      // head dismembered during the dying phase ends it
	inline float kHeadlessRollScale = 0.5f;       // roll strength factor when the dying phase starts without a head
	inline float kCrawlChance = 0.5f;             // chance a body lying face down crawls instead of twitching (wins over kTwitchChance)
	inline float kCrawlInterval = 2.4f;           // s: one reach + pull of an arm; the arms are half of it apart
	inline float kCrawlSpeed = 0.7f;              // peak forward speed of the body nudge while an arm pulls, physics units/s
	inline float kCrawlAccel = 40.0f;             // cap on the body nudge, units/s^2 (the dragged legs and arms add friction)
	inline float kCrawlDelay = 3.0f;              // s after the body is down (and turned face down) before the crawl starts
	inline float kCrawlReach = 0.75f;             // hand target ahead of the chest along the crawl direction
	inline float kCrawlSide = 0.28f;              // ... and out to its side
	inline float kCrawlLift = 0.1f;               // ... and above the chest centre while it reaches (elbow too)
	inline float kCrawlElbow = 0.6f;              // elbow target this far from the shoulder to the hand target (straighter past 0.5)
	inline float kCrawlArmStrength = 1.0f;        // scales the arm's reach/plant spring (brace gains)
	inline float kCrawlArmMaxHeight = 0.25f;      // hand or elbow higher than this above the chest: arm left to drop that frame
	inline float kCrawlStall = 0.25f;             // s a reaching hand may go without getting kCrawlStallGain closer before going limp
	inline float kCrawlMinPlant = 0.15f;          // hand must plant at least this far ahead of the chest to pull
	inline float kCrawlAnkleAccel = 150.0f;       // rad/s^2 ankle bend (toes tuck under) at the peak of a pull
	// Legs (with the opposite arm): while it reaches the thigh swings out along the ground and the knee bends, while it
	// pulls they straighten back. Peak joint speeds, rad/s (angle ≈ speed × fCrawlInterval / π).
	inline float kCrawlHipSpeed = 3.0f;
	inline float kCrawlKneeSpeed = 4.0f;
	// Head: turns toward the reaching arm and lifts with each pull; amplitudes in rad.
	inline float kCrawlHeadTurn = 0.8f;
	inline float kCrawlHeadLift = 0.6f;
	inline float kCrawlLimbAccel = 120.0f;        // rad/s^2 cap on matching the leg/head speeds
	inline float kCrawlLimbMaxTorque = 600.0f;    // ... and on the joint push (ankles too)
	inline float kCrawlTurnTime = 3.0f;           // s into the dying phase to get face down (side bodies are rolled), else no crawl
	inline float kTwitchDelay = 0.0f;             // s after the body is down before twitches and rolls start
	inline float kBurnRollDelay = 0.0f;           // s after a burning (knocked-down or burned-to-death) body is down before it rolls
	constexpr float kCrawlShoulder = 0.1f;        // shoulder estimate ahead of the chest centre
	constexpr float kCrawlStallGain = 0.02f;      // closer than this counts as progress for kCrawlStall
	constexpr float kCrawlRayLength = 1.0f;       // face-down ray from the stomach along the body's front ...
	constexpr float kCrawlDownDist = 0.4f;        // ... hitting the ground closer than this = face down
	constexpr float kCrawlSettle = 0.5f;          // s after a turn-over roll before the ray is trusted again
	inline float kDeathKnockbackScale = 0.4f;     // fraction of a gun's death knockback kept (1 = vanilla), not explosions
	inline float kExplosionStumbleChance = 0.0f;  // chance an explosion death still balances; otherwise it is flung
	inline Range kExplosionPush{ 6.0f, 10.0f };    // physics units/s: blast velocity given to every body part
	inline bool  kRagdollCollision = true;        // ragdolls collide with living NPCs and knock them over (else vanilla)
	inline bool  kSelfCollision = true;           // forearms/hands of dead and knocked-down ragdolls collide with their torso
	inline float kImpactSpeed = 1.0f;            // trunk speed at which a ragdoll knocks over a standing NPC (0 = off)
	inline float kImpactForce = 3.0f;             // knock force given to that NPC (engine explosion knockdown magnitude)
	inline float kExplosionLift = 0.5f;          // upward part of the blast push, as a fraction of it
	inline Range kDeathBraking{ 2.4f, 2.7f };     // horizontal braking of the dying body while its feet are planted, 1/s
	inline bool  kIniHotReload = false;           // [General] bEnableHotReload, read at game start only

	// Death stumble controller (v0.1.x). Gains are per second; caps are torque/force, converted with dt.
	constexpr float kJointKp = 20.0f;
	constexpr float kJointKd = 4.0f;
	constexpr float kJointMaxDeltaOmega = 3.0f;
	constexpr float kJointMaxTorque = 250.0f;
	constexpr float kTiltKp = 25.0f;           // soft on purpose: the body should sway, not lock
	constexpr float kTiltKd = 6.0f;
	constexpr float kYawKd = 8.0f;
	constexpr float kTiltMaxDeltaOmega = 1.5f; // rad/s per step
	constexpr float kHeightKp = 30.0f;
	constexpr float kHeightKd = 6.0f;
	constexpr float kStepTrigger = 0.08f;       // capture point this far from the feet starts a step
	constexpr float kStepTime = 0.3f;
	constexpr float kStepHeight = 0.2f;
	constexpr float kStepMaxLength = 0.45f;
	constexpr float kStepMinLength = 0.2f;      // shorter steps were hard to see
	constexpr float kSwingKneeBend = 50.0f;     // extra degrees the stepping knee folds mid-swing
	constexpr float kKneeMinStiffness = 1.0f;   // knees bend with at least this stiffness, even if fDeathStiffness is low
	constexpr float kKneeMinFade = 0.4f;        // knee drive never fades below this during the stumble
	constexpr float kStepCooldown = 0.12f;
	constexpr float kStepRayLift = 0.5f;        // landing ray starts this much higher than the foot rays (a step up)
	constexpr float kStepMaxRise = 0.5f;        // landing height change beyond this is not a step: keep the start height
	constexpr float kSwingKp = 20.0f;
	constexpr float kSwingMaxAccel = 120.0f;
	constexpr float kGroundTolerance = 0.2f;   // feet within this of the captured ground count as standing
	constexpr float kBalanceMaxAccel = 40.0f;  // per-body velocity change cap, units/s^2
	constexpr float kFlailArmSpeed = 14.0f;    // rad/s peak relative joint speed of a flailing arm per strength unit (legs: 60 %)
	constexpr float kFlailGain = 50.0f;        // 1/s: how tightly the joint follows the flail rhythm (beats joint friction)
	constexpr float kFlailMaxTorque = 300.0f;  // per flail strength unit
	constexpr float kAirborneClear = 0.35f;    // knockdown: feet and trunk this far above the ground = flying (flail, no brace)
	constexpr float kFlailGround = 0.45f;      // COM this close above the captured ground = landed, no flailing
	constexpr float kSupportBase = 0.4f;       // help is full while the COM is within this of the planted feet ...
	constexpr float kSupportFade = 0.4f;       // ... and fades to none over this much further (a body over a ledge tips)
	constexpr float kRayUp = 0.3f;             // ground rays start this far above the body (physics units) ...
	constexpr float kRayDown = 3.0f;           // ... and reach this far below it; no hit = ground at the ray end
	constexpr float kBraceLean = 0.35f;        // rad: the hands start reaching for the ground at this lean ...
	constexpr float kBraceFull = 0.7f;         // ... and reach fully from this lean
	constexpr float kBraceGround = 0.2f;       // ... until the COM is this close above the ground
	// Head guard, in the trunk's frame (up = pelvis to chest, forward = fall direction): hands in front of the face,
	// elbows (forearm bodies) forward of the chest and a bit wider, so the forearms curl up to protect the head.
	constexpr float kGuardHandUp = 0.30f;
	constexpr float kGuardHandForward = 0.22f;
	constexpr float kGuardHandSide = 0.10f;
	constexpr float kGuardElbowUp = 0.02f;
	constexpr float kGuardElbowForward = 0.32f;
	constexpr float kGuardElbowSide = 0.17f;
	constexpr float kBraceKp = 120.0f;         // hand → brace target spring, 1/s^2 (twice the grip's)
	constexpr float kBraceKd = 14.0f;          // ... damping on the hand's velocity relative to the chest, 1/s
	constexpr float kBraceMaxAccel = 300.0f;   // per-step velocity change cap for a bracing hand, units/s^2
	constexpr float kKnockWindow = 0.3f;       // s after death in which extra speed counts as weapon knockback
	constexpr float kFootFriction = 0.8f;      // sideways help is limited to this times the weight the feet carry
	constexpr float kSinkDepth = 0.25f;        // COM height target drops this much (physics units) as the knees fold
	constexpr float kGripKp = 60.0f;           // hand → wound spring, 1/s^2
	constexpr float kGripKd = 10.0f;           // ... damping on the hand's velocity relative to the wound, 1/s
	constexpr float kGripMaxAccel = 150.0f;    // per-step velocity change cap for the hand, units/s^2
	constexpr float kTwitchAccel = 500.0f;     // rad/s^2 relative joint acceleration during a twitch burst
	constexpr float kTwitchJolt = 4.0f;        // physics units/s limb jerk at the start of a twitch (equal/opposite)
	constexpr float kTwitchMinScale = 0.25f;   // lightest limb twitches at this fraction of the heaviest
	constexpr float kTwitchBurst = 0.15f;      // s: a spasm lasts this long, so joint friction cannot eat it
	constexpr float kTwitchMaxTorque = 300.0f; // per twitch strength unit
	constexpr float kDownTimeout = 1.5f;       // dying clock starts at the latest this long after the stumble
	// Rolling a body lying on its back onto its side lifts its weight about the shoulder edge: ~35–40 rad/s^2
	// is needed just to balance gravity, so the roll must be well above that.
	constexpr float kRollAccel = 80.0f;        // rad/s^2 at fRollStrength 1; ~90–100° per roll
	constexpr float kRollGap = 1.0f;           // s between rolls of the dying: gap .. 2 x gap
	constexpr float kRollBurst = 0.3f;         // s: long enough that ground friction cannot cancel it
	constexpr float kRollMaxAccel = 150.0f;    // per-body velocity change cap during a roll, units/s^2
	constexpr float kRollMinStrength = 0.4f;   // rolls keep at least this much strength through the dying phase
	constexpr float kLastHitWindow = 0.5f;     // a hit this recent drives the killing push

	constexpr float kGiveUpTilt = 1.2f;        // rad of trunk lean at which external help is gone (~70°)
	constexpr float kGiveUpRange = 0.5f;       // ... full help below kGiveUpTilt - kGiveUpRange (~40°)

	inline std::mutex                               g_mutex;
	inline std::vector<Stumble>                     g_stumbles;
	inline std::vector<Pending>                     g_pending;
	inline std::unordered_map<RE::TESFormID, float> g_lastTrigger;
	using LinearImpulse = void (*)(RE::hknpBSWorld*, std::uint32_t, const RE::hkVector4f*);
	inline LinearImpulse                            g_linearImpulse{};
	using AngularImpulse = void (*)(RE::hknpBSWorld*, std::uint32_t, const RE::hkVector4f*);
	inline AngularImpulse                           g_angularImpulse{};
	inline std::vector<Death>                       g_deaths;
	// NPCs whose head a hit dismembered or exploded (g_mutex). Gore heads keep their ragdoll joint, so UpdateDetached
	// does not see them.
	inline std::unordered_set<RE::TESFormID>        g_headless;
	inline std::vector<Death>                       g_knockdowns;  // living NPCs in an engine ragdoll: brace / flail only
	inline std::vector<RE::TESFormID>               g_pendingDeaths;
	inline std::vector<RE::TESFormID>               g_burningDeaths;  // on fire at KillImpl
	inline std::unordered_map<RE::TESFormID, Vec3>  g_deathMoveVel;  // actor velocity just before KillImpl, physics units/s
	inline std::unordered_map<RE::TESFormID, LastHit> g_lastHit;  // most recent hit per NPC, for the killing push

	// Read by the driveToPose hook (behavior-graph threads), written by the main thread.
	struct DriveState
	{
		RE::TESFormID formID{};
		float         start{};
		float         end{};
		bool          logged{ false };
		std::uintptr_t manager{ 0 };  // BSAnimationGraphManager, for the movement hook
		std::uint32_t calls{ 0 };     // driveToPose calls seen for this driver
		std::uint32_t injected{ 0 };  // calls where track 13 was written
		std::uint32_t movementQueries{ 0 };  // movement update asked "ragdoll active?"
		std::uint32_t movementCleared{ 0 };  // ... and the engine said yes, which we overrode
		bool          attackLogged{ false };  // logged a reaction during a melee attack
		bool          burning{ false };       // on fire: fixed loose stiffness, arms on their own soft entry
		float         burnProgress{ 0.0f };   // 0..1 towards the fall: the body loosens further
		std::uint64_t armBones{ 0 };          // ragdoll bone indices of the arms (bit per bone)
		std::uint64_t footBones{ 0 };         // ragdoll bone indices of the feet (bit per bone)
	};
	inline std::mutex                                     g_driveMutex;
	inline std::unordered_map<std::uintptr_t, DriveState> g_drives;  // key: hkbRagdollDriver*
	// Actors whose ragdoll joints do not fit their skeleton and could not be fitted (guarded by g_driveMutex): the
	// joints push the bodies apart (stretch, feet through the ground), so they get no hit reactions.
	inline std::unordered_set<RE::TESFormID>              g_mismatched;
	inline bool IsMismatched(RE::TESFormID a_id)
	{
		std::scoped_lock lock{ g_driveMutex };
		return g_mismatched.contains(a_id);
	}
	using DriveToPose = void (*)(void*, float, void*, void*);
	inline DriveToPose g_driveToPose{};
	using PostPhysics = void (*)(void*, void*, std::uint32_t, void*);
	inline PostPhysics g_postPhysics{};

	inline REL::THookVFT<bool(RE::bhkWorld*, std::uint32_t)>* g_worldUpdateHook = nullptr;

	// Havok's fixed step length while the step listener runs (Hooks.cpp), else 0.
	inline float g_stepDt = 0.0f;

	// Controller time step: the Havok step inside the listener, else the wall-clock gap since the last call.
	inline float ControlDt(float& a_last, float a_now)
	{
		const float dt = g_stepDt > 0.0f ? g_stepDt : a_last > 0.0f ? std::clamp(a_now - a_last, 0.001f, 0.05f) : 1.0f / 60.0f;
		a_last = a_now;
		return dt;
	}

	struct HkArrayView
	{
		std::uintptr_t data{};
		std::int32_t   size{};
		std::uint32_t  capacityAndFlags{};
	};
	static_assert(sizeof(HkArrayView) == 0x10);

	struct RagdollMap
	{
		std::uintptr_t ragdoll{};
		std::uint32_t  boneCount{};
		std::uint32_t  bodyCount{};
		std::uint32_t  constraintCount{};
		std::array<std::uint32_t, 64> bodyIDs{};
		std::array<std::uint32_t, 64> constraintIDs{};
		std::array<std::int32_t, 64>  boneToBody{};
		std::array<std::int16_t, 64>  parents{};
	};

	inline Vec3 Add(Vec3 a, Vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
	inline Vec3 Sub(Vec3 a, Vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
	inline Vec3 Scale(Vec3 a, float s) { return { a.x * s, a.y * s, a.z * s }; }
	inline float Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
	inline float Length(Vec3 a) { return std::sqrt(Dot(a, a)); }
	inline Vec3 Cross(Vec3 a, Vec3 b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
	inline Vec3 Unit(Vec3 a)
	{
		const float length = Length(a);
		return length > 1.0e-6f ? Scale(a, 1.0f / length) : Vec3{};
	}
	inline Quat Conjugate(Quat q) { return { -q.x, -q.y, -q.z, q.w }; }
	inline Vec3 Rotate(Quat q, Vec3 v)
	{
		const Vec3 axis{ q.x, q.y, q.z };
		const Vec3 cross{ axis.y * v.z - axis.z * v.y, axis.z * v.x - axis.x * v.z, axis.x * v.y - axis.y * v.x };
		const float dot = Dot(axis, v);
		const float scale = q.w * q.w - 0.5f;
		return {
			2.0f * (scale * v.x + dot * q.x + q.w * cross.x),
			2.0f * (scale * v.y + dot * q.y + q.w * cross.y),
			2.0f * (scale * v.z + dot * q.z + q.w * cross.z)
		};
	}

	struct MotionState
	{
		Quat orientation{};
		Vec3 linear{};
		Vec3 angular{};
		float inverseInertia[3]{};
		float inverseMass{};
	};

	// hknpBSWorld::m_worldLock (+0x6D8) is the engine's physics write lock; recursive for its owner.
	struct WorldWriteLock
	{
		explicit WorldWriteLock(RE::hknpBSWorld* a_world) :
			lock(&a_world->m_worldLock), held(lock->try_lock_write()) {}
		~WorldWriteLock()
		{
			if (held) lock->unlock_write();
		}
		WorldWriteLock(const WorldWriteLock&) = delete;
		WorldWriteLock& operator=(const WorldWriteLock&) = delete;
		RE::BSReadWriteLock* lock;
		bool                 held;
	};

	// Hook on the single DoHitMe call (task queue unpack, ID 1546751 + 0x921): see every hit before it is applied.
	using DoHitMe = void (*)(RE::Actor*, RE::HitData*);
	inline DoHitMe g_doHitMe{};

	inline Mat3 RotationFrom(const RE::hkRotationf& a_rotation)
	{
		Mat3 result{};
		for (int column = 0; column < 3; ++column)
			for (int row = 0; row < 3; ++row)
				result[row * 3 + column] = a_rotation.m_col[column][row];
		return result;
	}

	inline Mat3 Multiply(const Mat3& a, const Mat3& b)
	{
		Mat3 result{};
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c)
				for (int k = 0; k < 3; ++k)
					result[r * 3 + c] += a[r * 3 + k] * b[k * 3 + c];
		return result;
	}

	inline Mat3 Transpose(const Mat3& a)
	{
		Mat3 result{};
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c)
				result[r * 3 + c] = a[c * 3 + r];
		return result;
	}

	inline Vec3 Rotate(const Mat3& a, Vec3 v)
	{
		return {
			a[0] * v.x + a[1] * v.y + a[2] * v.z,
			a[3] * v.x + a[4] * v.y + a[5] * v.z,
			a[6] * v.x + a[7] * v.y + a[8] * v.z
		};
	}

	// Rotation by a_angle (rad) about unit axis a_axis (Rodrigues), same row-major convention as RotationFrom.
	inline Mat3 AxisAngle(Vec3 a_axis, float a_angle)
	{
		const float c = std::cos(a_angle), s = std::sin(a_angle), t = 1.0f - c;
		const float x = a_axis.x, y = a_axis.y, z = a_axis.z;
		return { t * x * x + c,     t * x * y - s * z, t * x * z + s * y,
			     t * x * y + s * z, t * y * y + c,     t * y * z - s * x,
			     t * x * z - s * y, t * y * z + s * x, t * z * z + c };
	}

	inline Mat3 RelativeRotation(const Mat3& a_parent, const Mat3& a_child)
	{
		return Multiply(Transpose(a_parent), a_child);
	}

	inline Vec3 RotationError(const Mat3& a_current, const Mat3& a_target)
	{
		const auto e = Multiply(a_current, Transpose(a_target));
		const float angle = std::acos(std::clamp((e[0] + e[4] + e[8] - 1.0f) * 0.5f, -1.0f, 1.0f));
		if (angle < 0.0001f) return {};
		const float sine = std::sin(angle);
		if (std::abs(sine) < 0.0001f) return { 1.0f, 0.0f, 0.0f };
		const float scale = angle / (2.0f * sine);
		return { (e[7] - e[5]) * scale, (e[2] - e[6]) * scale, (e[3] - e[1]) * scale };
	}

	// The actor movement update (ID 737625, 0x140D8CD80) asks the graph manager whether a ragdoll is active
	// (visitor ID 453608, 0x140DD4D70 → graph vtable slot 19: graph+0x3B8 set and ragdollInterface->IsAdded())
	// and zeroes the character-controller velocity if so. Our reaction must keep the ragdoll registered (the
	// same check also gates the per-frame driveToPose call), so only this movement query is answered "no"
	// for NPCs in a reaction.
	using RagdollActiveQuery = void (*)(void*, char**);
	inline RagdollActiveQuery g_ragdollActiveQuery{};

	// The graph's per-frame update (0x14161D9A0) also asks vtable slot 19 "ragdoll active?" first and, if so,
	// skips writing the frame's root motion. Movement then reuses the last delta every frame: the NPC keeps the
	// velocity it had at the hit (freeze at 0, sideways "ice skating", climbing). Slot 19 must stay true for the
	// pre-physics caller (it gates driveToPose), so only this one call site is answered "no", by return address.
	inline REL::THookVFT<bool(void*)>* g_ragdollActiveHook = nullptr;
	inline std::uintptr_t              g_graphUpdateReturn = 0;

	struct DeathSample { const BodyRef* ref; MotionState motion; Vec3 pos; };

	// Actor::KillImpl (vfunc 0x117, OG 0x140DB27B0). With ragdollInstant the engine goes straight to the death
	// ragdoll (0x140D70010) instead of performing the death action (default object +0x300, death animation).
	inline REL::THookVFT<void(RE::Actor*, RE::Actor*, float, bool, bool)>* g_killHook = nullptr;


	// Interiors use bhkWorld; exterior worldspaces use bhkWorldM (own vtable), so both are hooked.
	inline REL::THookVFT<bool(RE::bhkWorld*, std::uint32_t)>* g_worldMUpdateHook = nullptr;

	// ---- functions (see the .cpp named in each group) ----

	// Settings.cpp
	bool DebugLogEnabled();
	void LoadSettings();
	void PollSettings(float a_now);  // bEnableHotReload: re-reads the settings file when the file changes

	// Physics.cpp
	RE::bhkWorld* GetWorld(RE::Actor* a_actor);
	bool SafeRead(std::uintptr_t a_address, void* a_out, std::size_t a_size);
	float DecodePackedFloat(std::uint16_t packed);
	bool ReadMotionState(RE::hknpBSWorld* a_world, const RE::hknpBody& a_body, MotionState& a_out);
	Vec3 BodyPosition(const RE::hknpBody& a_body);
	bool ReadHkArray(std::uintptr_t a_object, std::size_t a_offset, std::uint32_t a_max, HkArrayView& a_out);
	bool FindRagdollMap(RE::Actor* a_actor, RE::hknpBSWorld* a_world, RagdollMap& a_out);
	bool FitJointsToSkeleton(RE::Actor* a_actor, RE::hknpBSWorld* a_world);
	std::int32_t FindWorldBody(RE::hknpBSWorld* a_world, std::uint32_t a_id);
	std::int32_t FindJointConstraint(RE::hknpBSWorld* a_world, const RagdollMap& a_map,
		std::uint32_t a_body, std::uint32_t a_parent);
	void ScanBodies(Stumble& a_stumble, RE::Actor* a_actor, RE::hknpBSWorld* a_world);
	bool Solve3(float matrix[3][3], Vec3 rhs, Vec3& result);
	void AddWorldInverse(const MotionState& a_motion, float a_matrix[3][3]);
	Vec3 ApplyWorldInverse(const MotionState& a_motion, Vec3 a_impulse);
	bool ConstraintCurrent(RE::hknpBSWorld* a_world, std::int32_t a_id, std::uint32_t a_body, std::uint32_t a_parent);
	bool SolveJointImpulse(const MotionState& a_child, const MotionState& a_parent, Vec3 a_desiredDelta, Vec3& a_impulse,
		float a_maxImpulse = 0.20f);
	void UpdateDetached(Stumble& a_stumble, RE::hknpBSWorld* a_world);
	void InitFlail(Stumble& a_stumble);
	std::int32_t FlailLimbs(RE::hknpBSWorld* a_world, Stumble& a_stumble, float a_time, float a_dt, float a_amount, float a_strength,
		char a_skipArm, char a_skipLeg);
	bool DriveFlailJoint(RE::hknpBSWorld* a_world, const Stumble& a_stumble, std::size_t a_joint, float a_time, float a_peak,
		float a_gain, float a_cap);
	std::pair<const DeathSample*, const DeathSample*> TrunkEnds(const std::vector<DeathSample>& a_samples);
	bool ApplyJointImpulse(RE::hknpBSWorld* a_world, const BodyRef& a_childRef, const BodyRef& a_parentRef, Vec3 a_desiredDelta,
		float a_maxImpulse);
	void ResolveImpulseFunction();
	void SetTreeMotionType(RE::NiAVObject* a_root, RE::hknpMotionPropertiesId::Preset a_type);
	void SetTreeCollisionLayer(RE::NiAVObject* a_root, RE::COL_LAYER a_layer);
	void UpdateCollisionFilter();
	void LearnArmTorsoParts(const Stumble& a_stumble, RE::hknpBSWorld* a_world);

	// Reaction.cpp
	std::uintptr_t FindRagdollDriver(RE::Actor* a_actor, std::uintptr_t* a_graph = nullptr);
	void ReadHitInfo(RE::Actor* a_actor, const RE::HitData& a_hit, Vec3& a_dir, std::uint32_t& a_hitBody);
	bool InLockedAnimation(RE::Actor* a_actor);
	const char* LockedReason(RE::Actor* a_actor);
	void LogSkipOnce(RE::Actor* a_actor, std::string_view a_where, std::string_view a_why);
	bool IsBurning(RE::Actor* a_actor, std::string* a_why = nullptr);  // a_why: which effect matched, for the log
	bool IsHumanoid(RE::Actor* a_actor);
	bool IsReallyDead(RE::Actor* a_actor);
	void ScanForBurning(float a_now);
	void ScanForKnockdowns(float a_now);
	void StepKnockdown(Death& a_down, RE::Actor* a_actor, float a_now);
	void ApplyBurn(Stumble& a_stumble, RE::Actor* a_actor, float a_now);
	void UpdateBurn(Stumble& a_stumble, RE::Actor* a_actor, float a_now);
	void SetAttackLock(Stumble& a_stumble, RE::Actor* a_actor, bool a_lock);
	bool PerformDefaultAction(RE::Actor* a_actor, RE::DEFAULT_OBJECT a_action);
	void StartBurnDown(RE::Actor* a_actor, float a_now);
	void KnockOverStanding(RE::bhkWorld* a_world, float a_now);
	void ReleaseAttackLocks();
	void SetBurnParalysis(Death& a_death, RE::Actor* a_actor, bool a_lock);
	void ReplaceDeath(Death&& a_death);
	bool QueueReaction(RE::Actor* a_actor, const RE::HitData& a_hit);
	void Hook_DoHitMe(RE::Actor* a_actor, RE::HitData* a_hit);
	void InstallHitHook();
	bool InjectRigidBodyControls(std::uintptr_t a_outputHolder, float a_stiffness, float a_weight, float a_dt, float a_catchUp,
		float a_armStiffness, DriveState& a_state);
	void Hook_DriveToPose(void* a_driver, float a_dt, void* a_context, void* a_output);
	void Hook_MovementRagdollQuery(void* a_manager, char** a_result);
	bool Hook_IsRagdollActive(void* a_graph);
	void InstallGraphUpdateHook();
	void InstallMovementHook();
	void InstallDriveToPoseHook();
	void InstallPostPhysicsHook();
	void ProcessPending();
	void ApplyHitPush(Stumble& a_stumble, RE::Actor* a_actor, float a_now);
	float HitPushScale(const RE::HitData& a_hit);
	float PushSpeed(const Stumble& a_stumble, const BodyRef& a_body, bool a_hitKnown, float a_bodyZ, float a_comZ);
	void EnforceNoCCLayer(Stumble& a_stumble, RE::Actor* a_actor);
	void TraceMovement(Stumble& a_stumble, RE::Actor* a_actor, float a_now);
	bool EndStumble(const Stumble& a_stumble, RE::Actor* a_actor);

	// Death.cpp
	void ApplyGrip(Death& a_stumble, RE::hknpBSWorld* npWorld, const std::vector<DeathSample>& samples, float dt, float a_strength);
	bool CastGround(RE::TESObjectCELL* a_cell, Vec3 a_pos, float& a_ground);
	void UpdateGround(Death& a_death, RE::Actor* a_actor, RE::bhkWorld* a_world);
	void CastCrawlRay(Death& a_death, RE::Actor* a_actor, RE::bhkWorld* a_world);
	void ApplyBrace(Death& a_death, RE::hknpBSWorld* npWorld, const std::vector<DeathSample>& samples, float dt, float a_lean, float a_scale,
		float a_ground);
	void StepDeath(Death& a_stumble, RE::Actor* a_actor, float a_now);
	void StepDying(Death& a_death, RE::Actor* a_actor, float a_now);
	void Hook_KillImpl(RE::Actor* a_actor, RE::Actor* a_attacker, float a_damage, bool a_sendEvent, bool a_ragdollInstant);
	void StartPendingDeaths(float a_now);

	// Hooks.cpp
	void OnWorldUpdate(RE::bhkWorld* a_world);
	bool Hook_WorldUpdate(RE::bhkWorld* a_world, std::uint32_t a_flags);
	bool Hook_WorldMUpdate(RE::bhkWorld* a_world, std::uint32_t a_flags);
	void InstallHooks();
	void ClearTracking();

	template <class T>
	bool ReadField(std::uintptr_t a_object, std::size_t a_offset, T& a_out)
	{
		return SafeRead(a_object + a_offset, &a_out, sizeof(a_out));
	}
}
