#include "Common.h"

namespace DynamicRagdollSystem
{

	// Written when the INI is missing. Keep in sync with res/DynamicRagdollSystem.ini.
	constexpr std::string_view kDefaultIni = R"INI(; DynamicRagdollSystem settings. Read at game start and again whenever a save is loaded (bDebugLog and
; bEnableIniHotReload only at game start).
; Values outside the allowed range are clamped. Missing values use the defaults shown.

[General]
; 1 = write diagnostic messages to DynamicRagdollSystem.log. Only read at game start.
bDebugLog=1
; 1 = re-read this file about once a second whenever it is saved, so changes apply without loading a save.
; 0 = only at game start and on save load. Only read at game start.
bEnableIniHotReload=0

[Reaction]
; Minimum damage of a single hit that triggers a reaction. Range 0-10000.
fMinDamage=5.0
; Hits on the same NPC closer together than this (seconds) are ignored, e.g. shotgun pellets. Range 0-60.
; A hit during a reaction restarts it.
fCooldown=0.1
; Seconds the body stays physical after a hit. Range 0.2-10.
fReactionTime=1.5
; Body stiffness right after the hit, as a fraction of normal (0 = fully limp, 1 = no give). Range 0-1.
fHitSoftness=0.1
; Seconds to firm back up to full stiffness. Range 0.05-10.
fRecoverTime=0.8
; Seconds at the end of the reaction to blend back to pure animation. Range 0-2.
fBlendOut=0.3
; Shove on the body part that was hit, along the shot direction. Range 0-50.
; (Melee and explosions have no hit body part: the whole upper body gets this.)
fHitPush=8.0
; Smaller shove on the rest of the upper body for the same hit. Range 0-50.
fBodyPush=5.5
; How fast the body may catch up with the animated pose. Range 0.5-100.
fMaxCatchUpSpeed=7.0
; While the NPC is in a melee attack (e.g. swinging a machete), the body never goes softer than this (fraction of
; normal stiffness). Lower = bigger reaction, but fast full-body swings can twist the body. At or below
; fHitSoftness it has no effect (the full normal reaction). Range 0-1.
fAttackSoftness=0.1
; Catch-up speed while in a melee attack, so the body keeps up with fast swings. Range 0.5-100.
fAttackCatchUpSpeed=9.0

[Burning]
; 1 = living humanoid NPCs (humans, ghouls, super mutants) on fire stop attacking, flail their arms and stagger
; around until the fire goes out. Detects base-game fire (flamer, Shishkebab, incendiary, Molotov, fire hazards).
bBurnReaction=1
; Body stiffness while burning, as a fraction of normal (lower = more wobbly; the NPC keeps its own animation).
; Range 0-1.
fBurnStiffness=0.3
; Arm stiffness while burning (near 0 = arms free to flail). Range 0-1.
fBurnArmStiffness=0.5
; Strength of the flailing arms. 0 = no flailing. Range 0-3.
fBurnFlail=1.4
; Random stagger shove on the upper body (physics units/s). 0 = no staggering. Range 0-20.
fBurnShove=2.5
; Seconds between stagger shoves, picked at random in this range. Range 0.1-10.
fBurnShoveIntervalMin=0.5
fBurnShoveIntervalMax=1.2
; Seconds of burning before the NPC loses its balance and falls (it gets looser and staggers harder on the way),
; picked per burn. It stays down writhing until the fire is out, then gets up. Both 0 = never falls. Range 0-60.
fBurnFallTimeMin=2.0
fBurnFallTimeMax=4.0
; Force of the fall (the engine's explosion knockdown push; it lies down and gets up by itself). Range 0-1000.
fBurnFallForce=3.0
; Burning NPC on the ground: strength of its rolls. 0 = no rolling. Range 0-3.
fBurnDownRoll=1.2
; Seconds between those rolls, picked at random in this range. Range 0.1-30.
fBurnDownRollIntervalMin=0.4
fBurnDownRollIntervalMax=0.8
; Seconds without fire before it may get up (fire effects can drop out for a moment between hits). Range 0-10.
fBurnOutGrace=1.0
; Longest it is kept on the ground, even if it keeps burning (fire hazards). Range 1-600.
fBurnDownMaxTime=60.0
; NPCs that die while on fire writhe with their own twitch and roll strengths (picked per death), instead of the
; [Death] fTwitchStrength/fRollStrength, and regardless of fTwitchChance. Both 0 = no writhing. Range 0-3.
fBurnDeathTwitchMin=2.6
fBurnDeathTwitchMax=3.0
fBurnDeathRollMin=1.3
fBurnDeathRollMax=1.8
; How long an NPC that died on fire keeps writhing before it is still (its dying phase, instead of fDyingTime),
; picked per death. Range 0-30.
fBurnDeathTimeMin=6.0
fBurnDeathTimeMax=10.0

[Death]
; Each death rolls its own values: chances are 0-1 (0 = never, 1 = always), and each Min/Max pair picks a
; random value in between (set both the same for a fixed value).

; 1 = NPCs skip the vanilla death animation and go straight to the death ragdoll.
bDisableDeathAnimation=1
; Chance the dead body tries to stay up for a moment (balance, stepping) before it collapses.
fDeathStumbleChance=1.0
; Chance one hand clutches the body part the killing shot hit, while stumbling and after falling.
fDeathGripChance=0.75
; Chance the body twitches and rolls on the ground after it falls.
fTwitchChance=0.8
; Seconds the dead body fights to stay up; its strength decays steadily over this time. Range 0.2-15.
fDeathStumbleTimeMin=2.3
fDeathStumbleTimeMax=2.6
; How much of its weight the dying body is held up with at first (0 = none, 1 = fully). Range 0-1.
fDeathSupportMin=0.6
fDeathSupportMax=0.8
; Joint stiffness of the dying body (lower = floppier). Range 0-3.
fDeathStiffness=0.2
; How hard the dying body is pushed back upright. Low lets it tip, so it steps to catch itself; high holds the
; torso up. Range 0-3.
fDeathUprightMin=0.4
fDeathUprightMax=0.6
; Degrees the knees fold by the end of the stumble, so the body sinks instead of stiff legs (0 = off). Range 0-120.
fDeathKneeBend=80.0
; Push of the killing hit, as a fraction of fHitPush/fBodyPush (a full-strength push topples the body at once). Range 0-3.
fDeathPushScale=0.4
; Fraction of a weapon's own death knockback that is kept (1 = vanilla). Strong guns otherwise launch the body
; too fast for the stumble to play. The NPC's movement before death and explosions are not affected. Range 0-1.
fDeathKnockbackScale=0.4
; Killing hits with at least this much damage blow the body away instead of stumbling (e.g. a Gauss rifle): full
; weapon knockback, no balance help, limbs flail in the air. The knockback log line shows each kill's damage.
; 0 = off. Range 0-100000.
fDeathFlingDamage=100.0
; Damage a dying body (clutching/twitching/rolling after the stumble) takes before it goes limp at once, summed over
; hits. 0 = off. Range 0-10000.
fDyingEndDamage=20.0
; 1 = a dying body goes limp at once when its head is dismembered (a head already off when the dying phase starts
; does not count). 0 = off.
bDyingEndOnHeadLoss=1
; Roll strength of a dying body whose head was already dismembered when the dying phase started, as a fraction of
; fRollStrength (1 = same). Range 0-1.
fHeadlessRollScale=0.5
; Free hands (not the one clutching the wound) reach for the ground when the dying body tips over, like catching
; a fall. 0 = off. Range 0-3.
fDeathBrace=2.5
; Speed (physics units/s; running is about 4-5) up to which the dying body gets full help to stay up and step.
; Faster deaths get less help and fall with their momentum. Range 0-20.
fDeathCatchSpeed=4.0
; Help fades to nothing over this much extra speed above fDeathCatchSpeed. Range 0-20.
fDeathCatchRange=6.0
; After the stumble, seconds the body keeps clutching the wound, twitching and rolling a little before it is
; fully limp (everything fades out over this time). 0 = limp right after the stumble. Range 0-30.
fDyingTimeMin=3.0
fDyingTimeMax=15.0
; Strength of the dying twitches. Range 0-3.
fTwitchStrengthMin=1.0
fTwitchStrengthMax=2.5
; Strength of the dying rolls (the body turning onto its side/back). Range 0-3.
fRollStrengthMin=0.5
fRollStrengthMax=1.5
; How quickly a dying body that is still on its feet loses its momentum (higher = stops sooner, lower = glides
; further). Range 0-20.
fDeathBrakingMin=2.4
fDeathBrakingMax=2.7
; Arms and legs swing around while a dying body flies through the air (died while moving fast, or knocked off its
; feet), until it lands. 0 = limp. Range 0-3.
fFlailStrengthMin=2.0
fFlailStrengthMax=3.0
; Chance an explosion death still tries to stay up like other deaths. Otherwise it is flung by the blast (no
; balance help, the limbs flail in the air). Range 0-1.
fExplosionStumbleChance=0.0
; Blast speed given to the whole body of a flung explosion death (physics units/s; running is about 4-5),
; picked per death. Range 0-50.
fExplosionPushMin=6.0
fExplosionPushMax=10.0
; Upward part of the blast, as a fraction of the push (0 = flat, 1 = as much up as away). Range 0-2.
fExplosionLift=0.5
; 1 = dead and knocked-down bodies collide with living NPCs instead of passing through them, and knock them over
; when they hit them fast enough (fRagdollImpactSpeed). Also lets walking NPCs bump into corpses on the floor.
; 0 = vanilla (no contact, no knock-over).
bRagdollCollideWithNPCs=1
; Dead or knocked-down bodies flying at least this fast (physics units/s; running is about 4-5) knock over
; standing NPCs they run into. 0 = no knock-over (the collision stays). Range 0-50.
fRagdollImpactSpeed=5.0
; How hard the NPC that was run into is knocked over (engine knockdown force, as fBurnFallForce). Range 0-1000.
fRagdollImpactForce=3.0
; 1 = forearms and hands of dead and knocked-down bodies collide with their own torso, so flailing arms do not pass
; through the body. Also makes dead bodies collide with each other. 0 = vanilla.
bRagdollSelfCollision=1
)INI";


	constexpr auto kIniPath = R"(.\Data\F4SE\Plugins\DynamicRagdollSystem.ini)";


	// bDebugLog: diagnostic logging (all REX::INFO), default on. Read before the logger starts.
	bool DebugLogEnabled()
	{
		return GetPrivateProfileIntA("General", "bDebugLog", 1, kIniPath) != 0;
	}


	void LoadSettings()
	{
		constexpr auto path = kIniPath;
		if (GetFileAttributesA(path) == INVALID_FILE_ATTRIBUTES) {
			if (std::FILE* file = nullptr; fopen_s(&file, path, "wb") == 0 && file) {
				std::fwrite(kDefaultIni.data(), 1, kDefaultIni.size(), file);
				std::fclose(file);
				REX::INFO("DynamicRagdollSystem: wrote default settings to {}"sv, path);
			}
		}
		static bool first = true;  // bEnableIniHotReload is only read at game start
		if (first) {
			first = false;
			kIniHotReload = GetPrivateProfileIntA("General", "bEnableIniHotReload", 0, path) != 0;
		}
		const auto read = [&](const char* a_key, float& a_value, float a_min, float a_max) {
			char buffer[64]{};
			GetPrivateProfileStringA("Reaction", a_key, "", buffer, sizeof(buffer), path);
			char* end = nullptr;
			const float value = std::strtof(buffer, &end);
			if (end != buffer && std::isfinite(value)) a_value = std::clamp(value, a_min, a_max);
		};
		read("fMinDamage", kMinDamage, 0.0f, 10000.0f);
		read("fCooldown", kCooldown, 0.0f, 60.0f);
		read("fReactionTime", kReactionTime, 0.2f, 10.0f);
		read("fHitSoftness", kHitSoftness, 0.0f, 1.0f);
		read("fRecoverTime", kRecoverTime, 0.05f, 10.0f);
		read("fBlendOut", kBlendOut, 0.0f, 2.0f);
		read("fHitPush", kHitPush, 0.0f, 50.0f);
		read("fBodyPush", kBodyPush, 0.0f, 50.0f);
		read("fMaxCatchUpSpeed", kMaxCatchUpSpeed, 0.5f, 100.0f);
		read("fAttackSoftness", kAttackSoftness, 0.0f, 1.0f);
		read("fAttackCatchUpSpeed", kAttackCatchUpSpeed, 0.5f, 100.0f);
		{
			const auto readBurn = [&](const char* a_key, float& a_value, float a_min, float a_max) {
				char buffer[64]{};
				GetPrivateProfileStringA("Burning", a_key, "", buffer, sizeof(buffer), path);
				char* end = nullptr;
				const float parsed = std::strtof(buffer, &end);
				if (end != buffer && std::isfinite(parsed)) a_value = std::clamp(parsed, a_min, a_max);
			};
			kBurnReaction = GetPrivateProfileIntA("Burning", "bBurnReaction", kBurnReaction ? 1 : 0, path) != 0;
			readBurn("fBurnStiffness", kBurnStiffness, 0.0f, 1.0f);
			readBurn("fBurnArmStiffness", kBurnArmStiffness, 0.0f, 1.0f);
			readBurn("fBurnFlail", kBurnFlail, 0.0f, 3.0f);
			readBurn("fBurnShove", kBurnShove, 0.0f, 20.0f);
			readBurn("fBurnShoveIntervalMin", kBurnShoveMin, 0.1f, 10.0f);
			readBurn("fBurnShoveIntervalMax", kBurnShoveMax, 0.1f, 10.0f);
			if (kBurnShoveMax < kBurnShoveMin) std::swap(kBurnShoveMin, kBurnShoveMax);
			readBurn("fBurnFallTimeMin", kBurnFallMin, 0.0f, 60.0f);
			readBurn("fBurnFallTimeMax", kBurnFallMax, 0.0f, 60.0f);
			readBurn("fBurnFallForce", kBurnFallForce, 0.0f, 1000.0f);
			readBurn("fBurnDeathTwitchMin", kBurnDeathTwitchMin, 0.0f, 3.0f);
			readBurn("fBurnDeathTwitchMax", kBurnDeathTwitchMax, 0.0f, 3.0f);
			readBurn("fBurnDeathRollMin", kBurnDeathRollMin, 0.0f, 3.0f);
			readBurn("fBurnDeathRollMax", kBurnDeathRollMax, 0.0f, 3.0f);
			if (kBurnDeathTwitchMax < kBurnDeathTwitchMin) std::swap(kBurnDeathTwitchMin, kBurnDeathTwitchMax);
			if (kBurnDeathRollMax < kBurnDeathRollMin) std::swap(kBurnDeathRollMin, kBurnDeathRollMax);
			readBurn("fBurnDeathTimeMin", kBurnDeathTimeMin, 0.0f, 30.0f);
			readBurn("fBurnDeathTimeMax", kBurnDeathTimeMax, 0.0f, 30.0f);
			if (kBurnDeathTimeMax < kBurnDeathTimeMin) std::swap(kBurnDeathTimeMin, kBurnDeathTimeMax);
			if (kBurnFallMax < kBurnFallMin) std::swap(kBurnFallMin, kBurnFallMax);
			readBurn("fBurnDownRoll", kBurnDownRoll, 0.0f, 3.0f);
			readBurn("fBurnDownRollIntervalMin", kBurnRollGapMin, 0.1f, 30.0f);
			readBurn("fBurnDownRollIntervalMax", kBurnRollGapMax, 0.1f, 30.0f);
			if (kBurnRollGapMax < kBurnRollGapMin) std::swap(kBurnRollGapMin, kBurnRollGapMax);
			readBurn("fBurnOutGrace", kBurnOutGrace, 0.0f, 10.0f);
			readBurn("fBurnDownMaxTime", kBurnDownMax, 1.0f, 600.0f);
		}
		kBlendOut = std::min(kBlendOut, kReactionTime);
		kDisableDeathAnimation = GetPrivateProfileIntA("Death", "bDisableDeathAnimation", kDisableDeathAnimation ? 1 : 0, path) != 0;
		const auto readDeath = [&](const std::string& a_key, float& a_value, float a_min, float a_max) {
			char buffer[64]{};
			GetPrivateProfileStringA("Death", a_key.c_str(), "", buffer, sizeof(buffer), path);
			char* end = nullptr;
			const float parsed = std::strtof(buffer, &end);
			if (end != buffer && std::isfinite(parsed)) a_value = std::clamp(parsed, a_min, a_max);
		};
		// Ranges: fXMin/fXMax; an old single fX key sets both, so older INIs keep working.
		const auto readRange = [&](const std::string& a_key, Range& a_range, float a_min, float a_max) {
			float single = a_range.min;
			readDeath(a_key, single, a_min, a_max);
			a_range = { single, single };
			readDeath(a_key + "Min", a_range.min, a_min, a_max);
			readDeath(a_key + "Max", a_range.max, a_min, a_max);
			if (a_range.max < a_range.min) std::swap(a_range.min, a_range.max);
		};
		readDeath("fDeathStumbleChance", kDeathStumbleChance, 0.0f, 1.0f);
		readDeath("fDeathGripChance", kDeathGripChance, 0.0f, 1.0f);
		readDeath("fTwitchChance", kTwitchChance, 0.0f, 1.0f);
		readRange("fDeathStumbleTime", kDeathStumbleTime, 0.2f, 15.0f);
		readRange("fDeathSupport", kDeathSupport, 0.0f, 1.0f);
		readRange("fDeathUpright", kDeathUpright, 0.0f, 3.0f);
		readRange("fDyingTime", kDyingTime, 0.0f, 30.0f);
		readRange("fTwitchStrength", kTwitchStrength, 0.0f, 3.0f);
		readRange("fRollStrength", kRollStrength, 0.0f, 3.0f);
		readRange("fDeathBraking", kDeathBraking, 0.0f, 20.0f);
		readRange("fFlailStrength", kFlailStrength, 0.0f, 3.0f);
		readDeath("fExplosionStumbleChance", kExplosionStumbleChance, 0.0f, 1.0f);
		readDeath("fDeathKnockbackScale", kDeathKnockbackScale, 0.0f, 1.0f);
		readDeath("fDeathFlingDamage", kDeathFlingDamage, 0.0f, 100000.0f);
		readDeath("fDyingEndDamage", kDyingEndDamage, 0.0f, 10000.0f);
		kDyingEndOnHeadLoss = GetPrivateProfileIntA("Death", "bDyingEndOnHeadLoss", kDyingEndOnHeadLoss ? 1 : 0, path) != 0;
		readDeath("fHeadlessRollScale", kHeadlessRollScale, 0.0f, 1.0f);
		readDeath("fDeathBrace", kDeathBrace, 0.0f, 3.0f);
		readRange("fExplosionPush", kExplosionPush, 0.0f, 50.0f);
		readDeath("fExplosionLift", kExplosionLift, 0.0f, 2.0f);
		kRagdollCollision = GetPrivateProfileIntA("Death", "bRagdollCollideWithNPCs", kRagdollCollision ? 1 : 0, path) != 0;
		kSelfCollision = GetPrivateProfileIntA("Death", "bRagdollSelfCollision", kSelfCollision ? 1 : 0, path) != 0;
		readDeath("fRagdollImpactSpeed", kImpactSpeed, 0.0f, 50.0f);
		readDeath("fRagdollImpactForce", kImpactForce, 0.0f, 1000.0f);
		readDeath("fDeathStiffness", kDeathStiffness, 0.0f, 3.0f);
		readDeath("fDeathPushScale", kDeathPushScale, 0.0f, 3.0f);
		readDeath("fDeathCatchSpeed", kCatchSpeed, 0.0f, 20.0f);
		readDeath("fDeathCatchRange", kCatchRange, 0.0f, 20.0f);
		readDeath("fDeathKneeBend", kDeathKneeBend, 0.0f, 120.0f);
		REX::INFO("DynamicRagdollSystem: settings: hotReload {}"sv, kIniHotReload);
		REX::INFO("DynamicRagdollSystem: settings: minDamage {} cooldown {} reaction {} softness {} recover {} blendOut {} push {} bodyPush {} catchUp {}"sv,
			kMinDamage, kCooldown, kReactionTime, kHitSoftness, kRecoverTime, kBlendOut, kHitPush, kBodyPush, kMaxCatchUpSpeed);
		REX::INFO("DynamicRagdollSystem: burn settings: reaction {} stiffness {} arms {} flail {} shove {} ({}-{}s) fall {}-{}s force {}; down roll {} ({}-{}s) grace {}s max {}s; death twitch {}-{} roll {}-{} time {}-{}s"sv,
			kBurnReaction, kBurnStiffness, kBurnArmStiffness, kBurnFlail, kBurnShove, kBurnShoveMin, kBurnShoveMax, kBurnFallMin, kBurnFallMax,
			kBurnFallForce, kBurnDownRoll, kBurnRollGapMin, kBurnRollGapMax, kBurnOutGrace, kBurnDownMax, kBurnDeathTwitchMin,
			kBurnDeathTwitchMax, kBurnDeathRollMin, kBurnDeathRollMax, kBurnDeathTimeMin, kBurnDeathTimeMax);
		REX::INFO("DynamicRagdollSystem: death settings: noDeathAnimation {} chances stumble {} grip {} twitch {}; stiffness {} pushScale {} catchSpeed {} catchRange {} kneeBend {}; dying ends at {} damage, on head loss {}, headless roll x{}"sv,
			kDisableDeathAnimation, kDeathStumbleChance, kDeathGripChance, kTwitchChance, kDeathStiffness, kDeathPushScale, kCatchSpeed, kCatchRange, kDeathKneeBend,
			kDyingEndDamage, kDyingEndOnHeadLoss, kHeadlessRollScale);
		REX::INFO("DynamicRagdollSystem: death ranges: time {}-{} support {}-{} upright {}-{} dying {}-{} twitch {}-{} roll {}-{} braking {}-{}"sv,
			kDeathStumbleTime.min, kDeathStumbleTime.max, kDeathSupport.min, kDeathSupport.max, kDeathUpright.min, kDeathUpright.max,
			kDyingTime.min, kDyingTime.max, kTwitchStrength.min, kTwitchStrength.max, kRollStrength.min, kRollStrength.max,
			kDeathBraking.min, kDeathBraking.max);
	}


	void PollSettings(float a_now)
	{
		if (!kIniHotReload) return;
		static float          nextCheck = 0.0f;
		static std::uint64_t  lastWrite = 0;
		if (a_now < nextCheck) return;
		nextCheck = a_now + 1.0f;
		WIN32_FILE_ATTRIBUTE_DATA data{};
		if (!GetFileAttributesExA(kIniPath, GetFileExInfoStandard, &data)) return;
		const std::uint64_t write = (std::uint64_t{ data.ftLastWriteTime.dwHighDateTime } << 32) | data.ftLastWriteTime.dwLowDateTime;
		if (lastWrite != 0 && write != lastWrite) {
			REX::INFO("DynamicRagdollSystem: INI changed, reloading"sv);
			LoadSettings();
		}
		lastWrite = write;
	}
}
