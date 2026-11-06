#include "Common.h"

#include <toml.hpp>

namespace DynamicRagdollSystem
{

	// Written when the file is missing. Keep in sync with res/DynamicRagdollSystem.toml.
	constexpr std::string_view kDefaultSettings = R"TOML(
# Values outside the allowed range are clamped. Missing values use defaults.

[General]
# true = write diagnostic messages to DynamicRagdollSystem.log. Only read at game start.
bDebugLog = true
# true = re-read this file about once a second whenever it is saved, so changes apply without loading a save.
bEnableHotReload = false

[Reaction]
# true = hit reactions only on humans, ghouls, feral ghouls and super mutants.
bHumanoidOnly = false
# Minimum damage of a single hit that triggers a reaction. Range 0-10000.
fMinDamage = 5.0
# Hits on the same NPC closer together than this (seconds) are ignored, e.g. shotgun pellets. Range 0-60.
# A hit during a reaction restarts it.
fCooldown = 0.1
# Seconds the body stays physical after a hit. Range 0.2-10.
fReactionTime = 1.5
# Body stiffness right after the hit, as a fraction of normal (0 = fully limp, 1 = no give). Range 0-1.
fHitSoftness = 0.1
# Seconds to firm back up to full stiffness. Range 0.05-10.
fRecoverTime = 0.8
# Seconds at the end of the reaction to blend back to pure animation. Range 0-2.
fBlendOut = 0.3
# Shove on the body part that was hit, along the shot direction. Range 0-50.
# (Melee and explosions have no hit body part: the whole upper body gets this.)
fHitPush = 8.0
# Shove on the body parts one joint away from the one that was hit; halved for each further joint, none past
# three joints. Range 0-50.
fBodyPush = 5.5
# Both shoves scale with the damage of the hit: fPushDamageRef damage gives them as set, half that damage half
# the shove, and so on, kept within fPushScaleMin-fPushScaleMax. 0 = same shove for every hit. Range 0-10000.
fPushDamageRef = 25.0
# Range 0-10.
fPushScaleMin = 0.5
fPushScaleMax = 2.0
# Extra shove factor for melee power attacks, and for blocked hits. Range 0-10.
fPushPowerAttackMult = 1.5
fPushBlockedMult = 0.6
# How fast the body may catch up with the animated pose. Range 0.5-100.
fMaxCatchUpSpeed = 7.0
# While the NPC is in a melee attack (e.g. swinging a machete), the body never goes softer than this (fraction of
# normal stiffness). Lower = bigger reaction, but fast full-body swings can twist the body. At or below
# fHitSoftness it has no effect (the full normal reaction). Range 0-1.
fAttackSoftness = 0.1
# Catch-up speed while in a melee attack, so the body keeps up with fast swings. Range 0.5-100.
fAttackCatchUpSpeed = 9.0

[Burning]
# true = living humanoid NPCs (humans, ghouls, super mutants) on fire stop attacking, flail their arms and stagger
# around until the fire goes out. Detects base-game fire (flamer, Shishkebab, incendiary, Molotov, fire hazards).
bBurnReaction = true
# Body stiffness while burning, as a fraction of normal (lower = more wobbly; the NPC keeps its own animation). Range 0-1.
fBurnStiffness = 0.3
# Arm stiffness while burning (near 0 = arms free to flail). Range 0-1.
fBurnArmStiffness = 0.5
# Strength of the flailing arms. 0 = no flailing. Range 0-3.
fBurnFlail = 1.4
# Random stagger shove on the upper body (physics units/s). 0 = no staggering. Range 0-20.
fBurnShove = 2.5
# Seconds between stagger shoves, picked at random in this range. Range 0.1-10.
fBurnShoveIntervalMin = 0.5
fBurnShoveIntervalMax = 1.2
# Seconds of burning before the NPC loses its balance and falls (it gets looser and staggers harder on the way),
# picked per burn. It stays down writhing until the fire is out, then gets up. Both 0 = never falls. Range 0-60.
fBurnFallTimeMin = 2.0
fBurnFallTimeMax = 4.0
# Force of the fall (the engine's explosion knockdown push; it lies down and gets up by itself). Range 0-1000.
fBurnFallForce = 2.5
# Burning NPC on the ground: strength of its rolls. 0 = no rolling. Range 0-3.
fBurnDownRoll = 1.2
# Seconds between those rolls, picked at random in this range. Range 0.1-30.
fBurnDownRollIntervalMin = 0.4
fBurnDownRollIntervalMax = 0.8
# true = burn rolls (burning on the ground, burned to death) also swing the arms with the trunk; false = the arms follow the shoulders.
bBurnRollArms = true
# Seconds after a burning body (knocked down, or burned to death) is down before it starts rolling. Range 0-10.
fBurnRollDelay = 0.0
# Seconds without fire before it may get up (fire effects can drop out for a moment between hits). Range 0-10.
fBurnOutGrace = 1.0
# Longest it is kept on the ground, even if it keeps burning (fire hazards). Range 1-600.
fBurnDownMaxTime = 45.0
# NPCs that die while on fire writhe with their own twitch and roll strengths (picked per death), instead of the
# [Death] fTwitchStrength/fRollStrength, and regardless of fTwitchChance. Both 0 = no writhing. Range 0-3.
fBurnDeathTwitchMin = 2.6
fBurnDeathTwitchMax = 3.0
fBurnDeathRollMin = 1.3
fBurnDeathRollMax = 1.8
# How long an NPC that died on fire keeps writhing before it is still (its dying phase, instead of fDyingTime),
# picked per death. Range 0-30.
fBurnDeathTimeMin = 6.0
fBurnDeathTimeMax = 10.0

[Death]
# Each death rolls its own values: chances are 0-1 (0 = never, 1 = always), and each Min/Max pair picks a
# random value in between (set both the same for a fixed value).

# true = NPCs skip the vanilla death animation and go straight to the death ragdoll.
bDisableDeathAnimation = true
# Chance the dead body tries to stay up for a moment (balance, stepping) before it collapses.
fDeathStumbleChance = 1.0
# Chance one hand clutches the body part the killing shot hit, while stumbling and after falling.
fDeathGripChance = 0.7
# Chance the body twitches and rolls on the ground after it falls.
fTwitchChance = 0.8
# Seconds the dead body fights to stay up; its strength decays steadily over this time. Range 0.2-15.
fDeathStumbleTimeMin = 2.3
fDeathStumbleTimeMax = 2.6
# How much of its weight the dying body is held up with at first (0 = none, 1 = fully). Range 0-1.
fDeathSupportMin = 0.6
fDeathSupportMax = 0.8
# Joint stiffness of the dying body (lower = floppier). Range 0-3.
fDeathStiffness = 0.2
# How hard the dying body is pushed back upright. Low lets it tip, so it steps to catch itself; high holds the
# torso up. Range 0-3.
fDeathUprightMin = 0.4
fDeathUprightMax = 0.6
# Degrees the knees fold by the end of the stumble, so the body sinks instead of stiff legs (0 = off). Range 0-120.
fDeathKneeBend = 80.0
# Push of the killing hit, as a fraction of fHitPush/fBodyPush (a full-strength push topples the body at once). Range 0-3.
fDeathPushScale = 0.4
# Fraction of a weapon's own death knockback that is kept (1 = vanilla). Strong guns otherwise launch the body
# too fast for the stumble to play. The NPC's movement before death and explosions are not affected. Range 0-1.
fDeathKnockbackScale = 0.4
# Killing hits with at least this much damage blow the body away instead of stumbling (e.g. a Gauss rifle): full
# weapon knockback, no balance help, limbs flail in the air. The knockback log line shows each kill's damage.
# 0 = off. Range 0-100000.
fDeathFlingDamage = 75.0
# Damage a dying body (clutching/twitching/rolling after the stumble) takes before it goes limp at once, summed over
# hits. 0 = off. Range 0-10000.
fDyingEndDamage = 20.0
# true = a dying body goes limp at once when its head is dismembered (a head already off when the dying phase starts
# does not count). false = off.
bDyingEndOnHeadLoss = true
# Roll strength of a dying body whose head was already dismembered when the dying phase started, as a fraction of
# fRollStrength (1 = same). Range 0-1.
fHeadlessRollScale = 0.5
# Free hands (not the one clutching the wound) reach for the ground when the dying body tips over, like catching
# a fall. 0 = off. Range 0-3.
fDeathBrace = 2.5
# Speed (physics units/s; running is about 4-5) up to which the dying body gets full help to stay up and step.
# Faster deaths get less help and fall with their momentum. Range 0-20.
fDeathCatchSpeed = 4.0
# Help fades to nothing over this much extra speed above fDeathCatchSpeed. Range 0-20.
fDeathCatchRange = 6.0
# After the stumble, seconds the body keeps clutching the wound, twitching and rolling a little before it is
# fully limp (everything fades out over this time). 0 = limp right after the stumble. Range 0-30.
fDyingTimeMin = 3.0
fDyingTimeMax = 15.0
# Strength of the dying twitches. Range 0-3.
fTwitchStrengthMin = 1.0
fTwitchStrengthMax = 2.5
# Strength of the dying rolls (the body turning onto its side/back). Range 0-3.
fRollStrengthMin = 0.5
fRollStrengthMax = 1.5
# How hard a dying roll turns the trunk (rad/s^2 at roll strength 1). Also the crawl turn-over. Range 0-1000.
fRollAccel = 160.0
# true = dying rolls also swing the arms with the trunk (can fling a hand and drag the body over); false = the arms follow the shoulders.
bDeathRollArms = false
# Seconds after the body is down before its twitches and rolls start. Range 0-10.
fTwitchDelay = 0.0
# How quickly a dying body that is still on its feet loses its momentum (higher = stops sooner, lower = glides
# further). Range 0-20.
fDeathBrakingMin = 2.4
fDeathBrakingMax = 2.7
# Arms and legs swing around while a dying body flies through the air (died while moving fast, or knocked off its
# feet), until it lands. 0 = limp. Range 0-3.
fFlailStrengthMin = 2.0
fFlailStrengthMax = 3.0
# Chance an explosion death still tries to stay up like other deaths. Otherwise it is flung by the blast (no
# balance help, the limbs flail in the air). Range 0-1.
fExplosionStumbleChance = 0.0
# Blast speed given to the whole body of a flung explosion death (physics units/s; running is about 4-5),
# picked per death. Range 0-50.
fExplosionPushMin = 6.0
fExplosionPushMax = 10.0
# Upward part of the blast, as a fraction of the push (0 = flat, 1 = as much up as away). Range 0-2.
fExplosionLift = 0.5
# true = dead and knocked-down bodies collide with living NPCs instead of passing through them, and knock them over
# when they hit them fast enough (fRagdollImpactSpeed). Also lets walking NPCs bump into corpses on the floor.
# false = vanilla (no contact, no knock-over).
bRagdollCollideWithNPCs = true
# Dead or knocked-down bodies flying at least this fast (physics units/s; running is about 4-5) knock over
# standing NPCs they run into. 0 = no knock-over (the collision stays). Range 0-50.
fRagdollImpactSpeed = 1.0
# How hard the NPC that was run into is knocked over (engine knockdown force, as fBurnFallForce). Range 0-1000.
fRagdollImpactForce = 3.0
# true = forearms and hands of dead and knocked-down bodies collide with their own torso, so flailing arms do not pass
# through the body. Also makes dead bodies collide with each other. false = vanilla.
bRagdollSelfCollision = true

[Crawl]
# Dying bodies lying face down may crawl instead of twitching.
# Chance a dying body crawls instead of twitching and rolling, if it lies face down when the dying phase starts
# (otherwise fTwitchChance decides as usual). Never after a head shot or for burned deaths. Lasts fCrawlTime;
# fDyingEndDamage and bDyingEndOnHeadLoss end it early too. Range 0-1.
fCrawlChance = 0.3
# Forward speed the crawl reaches during each pull (physics units/s; running is about 4-5). Range 0-5.
fCrawlSpeed = 0.7
# Seconds for one reach-and-pull of an arm (the arms alternate, half of it apart). Lower = faster crawl. Range 0.4-5.
fCrawlInterval = 2.4
# Seconds after the body is down (and turned face down) before the crawl starts. Range 0-10.
fCrawlDelay = 3.0
# Seconds the crawl lasts from its start (after fCrawlDelay), fading out; picked at random in this range. Used
# instead of fDyingTime for crawling bodies. Range 0.5-60.
fCrawlTimeMin = 10.0
fCrawlTimeMax = 20.0
# Body push: peak forward speed is fCrawlSpeed; this caps how fast it gets there (units/s^2; the dragged limbs add
# friction, so too low and the body does not move). Range 0-200.
fCrawlPushAccel = 40.0
# Where the hand reaches to, from the chest: ahead, out to the side, and lifted above it (physics units, ~70 game
# units each). Further out reads as more outstretched but stalls more at the shoulder. Ranges 0-1.5, 0-1, 0-0.5.
fCrawlReach = 0.75
fCrawlSide = 0.28
fCrawlLift = 0.1
# Elbow target, as the fraction of the way from the shoulder to the hand target (0.5 = bent, higher = straighter
# arm). Range 0-1.
fCrawlElbow = 0.6
# Strength of the arm's reach and plant (1 = default). Range 0-5.
fCrawlArmStrength = 1.0
# A hand or elbow higher than this above the chest is left to drop that frame instead of being driven (stops an
# arm locking up over the back). Range 0-2.
fCrawlArmMaxHeight = 0.25
# Seconds a reaching hand may stop getting closer (a joint limit, another body in the way) before that arm goes limp
# for the rest of the reach. Range 0.05-2.
fCrawlStallTime = 0.25
# A hand must plant at least this far ahead of the chest to pull (physics units). Range 0-1.
fCrawlMinPlant = 0.15
# Ankle bend (toes tuck under) with each pull, rad/s^2. 0 = off. Range 0-2000.
fCrawlAnkleBend = 150.0
# Legs move with the opposite arm: thigh swings out along the ground and the knee bends while that arm reaches, and
# straighten while it pulls. Peak joint speeds in rad/s (angle is about speed x fCrawlInterval / 3.14). 0 = off.
# Range 0-20.
fCrawlHipSpeed = 3.0
fCrawlKneeSpeed = 4.0
# Head turns toward the reaching arm and lifts with each pull, in radians (0.5 = ~29 degrees). 0 = off. Range 0-1.5.
fCrawlHeadTurn = 0.8
fCrawlHeadLift = 0.6
# How hard legs, head and ankles may be pushed to follow that motion: acceleration cap (rad/s^2) and joint push cap.
# Raise if they barely move. Ranges 0-2000, 0-5000.
fCrawlLimbAccel = 120.0
fCrawlLimbTorque = 600.0
# Seconds into the dying phase to get face down (a body on its side is rolled over), else no crawl. Range 0-10.
fCrawlTurnTime = 3.0
)TOML";


	constexpr auto kSettingsPath = R"(.\Data\F4SE\Plugins\DynamicRagdollSystem.toml)";

	// Parsed settings file; an empty table if it is missing or broken (every setting keeps its current value).
	static toml::value ParseSettings()
	{
		try {
			return toml::parse(kSettingsPath);
		} catch (const std::exception& e) {
			REX::WARN("DynamicRagdollSystem: settings file not read, keeping current values: {}"sv, e.what());
			return toml::value{ toml::table{} };
		}
	}

	// The value at [a_section] a_key, or nullptr.
	static const toml::value* Find(const toml::value& a_root, std::string_view a_section, std::string_view a_key)
	{
		if (!a_root.is_table() || !a_root.contains(std::string{ a_section })) return nullptr;
		const auto& section = a_root.at(std::string{ a_section });
		if (!section.is_table() || !section.contains(std::string{ a_key })) return nullptr;
		return &section.at(std::string{ a_key });
	}

	// Booleans also accept 0/1. Missing or wrong type: a_value unchanged.
	static void ReadBool(const toml::value& a_root, std::string_view a_section, std::string_view a_key, bool& a_value)
	{
		if (const auto* v = Find(a_root, a_section, a_key)) {
			if (v->is_boolean()) a_value = v->as_boolean();
			else if (v->is_integer()) a_value = v->as_integer() != 0;
		}
	}

	// Numbers (integer or float), clamped. Returns whether the key was there.
	static bool ReadFloat(const toml::value& a_root, std::string_view a_section, std::string_view a_key, float& a_value, float a_min, float a_max)
	{
		const auto* v = Find(a_root, a_section, a_key);
		double parsed = 0.0;
		if (v && v->is_floating()) parsed = v->as_floating();
		else if (v && v->is_integer()) parsed = static_cast<double>(v->as_integer());
		else return false;
		if (!std::isfinite(parsed)) return false;
		a_value = std::clamp(static_cast<float>(parsed), a_min, a_max);
		return true;
	}


	// bDebugLog: diagnostic logging (all REX::INFO), default on. Read before the logger starts.
	bool DebugLogEnabled()
	{
		bool enabled = true;
		ReadBool(ParseSettings(), "General", "bDebugLog", enabled);
		return enabled;
	}


	void LoadSettings()
	{
		if (GetFileAttributesA(kSettingsPath) == INVALID_FILE_ATTRIBUTES) {
			if (std::FILE* file = nullptr; fopen_s(&file, kSettingsPath, "wb") == 0 && file) {
				std::fwrite(kDefaultSettings.data(), 1, kDefaultSettings.size(), file);
				std::fclose(file);
				REX::INFO("DynamicRagdollSystem: wrote default settings to {}"sv, kSettingsPath);
			}
		}
		const toml::value root = ParseSettings();
		static bool first = true;  // bEnableHotReload is only read at game start
		if (first) {
			first = false;
			ReadBool(root, "General", "bEnableHotReload", kIniHotReload);
		}
		const auto read = [&](const char* a_key, float& a_value, float a_min, float a_max) { ReadFloat(root, "Reaction", a_key, a_value, a_min, a_max); };
		ReadBool(root, "Reaction", "bHumanoidOnly", kReactHumanoidOnly);
		read("fMinDamage", kMinDamage, 0.0f, 10000.0f);
		read("fCooldown", kCooldown, 0.0f, 60.0f);
		read("fReactionTime", kReactionTime, 0.2f, 10.0f);
		read("fHitSoftness", kHitSoftness, 0.0f, 1.0f);
		read("fRecoverTime", kRecoverTime, 0.05f, 10.0f);
		read("fBlendOut", kBlendOut, 0.0f, 2.0f);
		read("fHitPush", kHitPush, 0.0f, 50.0f);
		read("fBodyPush", kBodyPush, 0.0f, 50.0f);
		read("fPushDamageRef", kPushDamageRef, 0.0f, 10000.0f);
		read("fPushScaleMin", kPushScaleMin, 0.0f, 10.0f);
		read("fPushScaleMax", kPushScaleMax, 0.0f, 10.0f);
		if (kPushScaleMax < kPushScaleMin) std::swap(kPushScaleMin, kPushScaleMax);
		read("fPushPowerAttackMult", kPushPowerAttackMult, 0.0f, 10.0f);
		read("fPushBlockedMult", kPushBlockedMult, 0.0f, 10.0f);
		read("fMaxCatchUpSpeed", kMaxCatchUpSpeed, 0.5f, 100.0f);
		read("fAttackSoftness", kAttackSoftness, 0.0f, 1.0f);
		read("fAttackCatchUpSpeed", kAttackCatchUpSpeed, 0.5f, 100.0f);
		{
			const auto readBurn = [&](const char* a_key, float& a_value, float a_min, float a_max) { ReadFloat(root, "Burning", a_key, a_value, a_min, a_max); };
			ReadBool(root, "Burning", "bBurnReaction", kBurnReaction);
		ReadBool(root, "Burning", "bBurnRollArms", kBurnRollArms);
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
			readBurn("fBurnRollDelay", kBurnRollDelay, 0.0f, 10.0f);
			readBurn("fBurnDownRollIntervalMin", kBurnRollGapMin, 0.1f, 30.0f);
			readBurn("fBurnDownRollIntervalMax", kBurnRollGapMax, 0.1f, 30.0f);
			if (kBurnRollGapMax < kBurnRollGapMin) std::swap(kBurnRollGapMin, kBurnRollGapMax);
			readBurn("fBurnOutGrace", kBurnOutGrace, 0.0f, 10.0f);
			readBurn("fBurnDownMaxTime", kBurnDownMax, 1.0f, 600.0f);
		}
		kBlendOut = std::min(kBlendOut, kReactionTime);
		ReadBool(root, "Death", "bDisableDeathAnimation", kDisableDeathAnimation);
		const auto readDeath = [&](const std::string& a_key, float& a_value, float a_min, float a_max) {
			return ReadFloat(root, "Death", a_key, a_value, a_min, a_max);
		};
		const auto readCrawl = [&](const std::string& a_key, float& a_value, float a_min, float a_max) {
			return ReadFloat(root, "Crawl", a_key, a_value, a_min, a_max);
		};
		// Ranges: fXMin/fXMax. Missing keys keep the defaults.
		const auto readRangeWith = [&](auto a_read, const std::string& a_key, Range& a_range, float a_min, float a_max) {
			a_read(a_key + "Min", a_range.min, a_min, a_max);
			a_read(a_key + "Max", a_range.max, a_min, a_max);
			if (a_range.max < a_range.min) std::swap(a_range.min, a_range.max);
		};
		const auto readRange = [&](const std::string& a_key, Range& a_range, float a_min, float a_max) {
			readRangeWith(readDeath, a_key, a_range, a_min, a_max);
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
		readDeath("fRollAccel", kRollAccel, 0.0f, 1000.0f);
		ReadBool(root, "Death", "bDeathRollArms", kDeathRollArms);
		readRange("fDeathBraking", kDeathBraking, 0.0f, 20.0f);
		readRange("fFlailStrength", kFlailStrength, 0.0f, 3.0f);
		readDeath("fExplosionStumbleChance", kExplosionStumbleChance, 0.0f, 1.0f);
		readDeath("fDeathKnockbackScale", kDeathKnockbackScale, 0.0f, 1.0f);
		readDeath("fDeathFlingDamage", kDeathFlingDamage, 0.0f, 100000.0f);
		readDeath("fDyingEndDamage", kDyingEndDamage, 0.0f, 10000.0f);
		ReadBool(root, "Death", "bDyingEndOnHeadLoss", kDyingEndOnHeadLoss);
		readDeath("fHeadlessRollScale", kHeadlessRollScale, 0.0f, 1.0f);
		readCrawl("fCrawlChance", kCrawlChance, 0.0f, 1.0f);
		readCrawl("fCrawlSpeed", kCrawlSpeed, 0.0f, 5.0f);
		readCrawl("fCrawlInterval", kCrawlInterval, 0.4f, 5.0f);
		readCrawl("fCrawlDelay", kCrawlDelay, 0.0f, 10.0f);
		readRangeWith(readCrawl, "fCrawlTime", kCrawlTime, 0.5f, 60.0f);
		readCrawl("fCrawlPushAccel", kCrawlAccel, 0.0f, 200.0f);
		readCrawl("fCrawlReach", kCrawlReach, 0.0f, 1.5f);
		readCrawl("fCrawlSide", kCrawlSide, 0.0f, 1.0f);
		readCrawl("fCrawlLift", kCrawlLift, 0.0f, 0.5f);
		readCrawl("fCrawlElbow", kCrawlElbow, 0.0f, 1.0f);
		readCrawl("fCrawlArmStrength", kCrawlArmStrength, 0.0f, 5.0f);
		readCrawl("fCrawlArmMaxHeight", kCrawlArmMaxHeight, 0.0f, 2.0f);
		readCrawl("fCrawlStallTime", kCrawlStall, 0.05f, 2.0f);
		readCrawl("fCrawlMinPlant", kCrawlMinPlant, 0.0f, 1.0f);
		readCrawl("fCrawlAnkleBend", kCrawlAnkleAccel, 0.0f, 2000.0f);
		readCrawl("fCrawlHipSpeed", kCrawlHipSpeed, 0.0f, 20.0f);
		readCrawl("fCrawlKneeSpeed", kCrawlKneeSpeed, 0.0f, 20.0f);
		readCrawl("fCrawlHeadTurn", kCrawlHeadTurn, 0.0f, 1.5f);
		readCrawl("fCrawlHeadLift", kCrawlHeadLift, 0.0f, 1.5f);
		readCrawl("fCrawlLimbAccel", kCrawlLimbAccel, 0.0f, 2000.0f);
		readCrawl("fCrawlLimbTorque", kCrawlLimbMaxTorque, 0.0f, 5000.0f);
		readCrawl("fCrawlTurnTime", kCrawlTurnTime, 0.0f, 10.0f);
		readDeath("fTwitchDelay", kTwitchDelay, 0.0f, 10.0f);
		readDeath("fDeathBrace", kDeathBrace, 0.0f, 3.0f);
		readRange("fExplosionPush", kExplosionPush, 0.0f, 50.0f);
		readDeath("fExplosionLift", kExplosionLift, 0.0f, 2.0f);
		ReadBool(root, "Death", "bRagdollCollideWithNPCs", kRagdollCollision);
		ReadBool(root, "Death", "bRagdollSelfCollision", kSelfCollision);
		readDeath("fRagdollImpactSpeed", kImpactSpeed, 0.0f, 50.0f);
		readDeath("fRagdollImpactForce", kImpactForce, 0.0f, 1000.0f);
		readDeath("fDeathStiffness", kDeathStiffness, 0.0f, 3.0f);
		readDeath("fDeathPushScale", kDeathPushScale, 0.0f, 3.0f);
		readDeath("fDeathCatchSpeed", kCatchSpeed, 0.0f, 20.0f);
		readDeath("fDeathCatchRange", kCatchRange, 0.0f, 20.0f);
		readDeath("fDeathKneeBend", kDeathKneeBend, 0.0f, 120.0f);
		REX::INFO("DynamicRagdollSystem: settings: hotReload {}"sv, kIniHotReload);
		REX::INFO("DynamicRagdollSystem: settings: minDamage {} cooldown {} reaction {} softness {} recover {} blendOut {} push {} bodyPush {} catchUp {} pushDamageRef {} pushScale {}-{} powerAttack {} blocked {}"sv,
			kMinDamage, kCooldown, kReactionTime, kHitSoftness, kRecoverTime, kBlendOut, kHitPush, kBodyPush, kMaxCatchUpSpeed,
			kPushDamageRef, kPushScaleMin, kPushScaleMax, kPushPowerAttackMult, kPushBlockedMult);
		REX::INFO("DynamicRagdollSystem: burn settings: reaction {} stiffness {} arms {} flail {} shove {} ({}-{}s) fall {}-{}s force {}; down roll {} ({}-{}s) grace {}s max {}s; death twitch {}-{} roll {}-{} time {}-{}s"sv,
			kBurnReaction, kBurnStiffness, kBurnArmStiffness, kBurnFlail, kBurnShove, kBurnShoveMin, kBurnShoveMax, kBurnFallMin, kBurnFallMax,
			kBurnFallForce, kBurnDownRoll, kBurnRollGapMin, kBurnRollGapMax, kBurnOutGrace, kBurnDownMax, kBurnDeathTwitchMin,
			kBurnDeathTwitchMax, kBurnDeathRollMin, kBurnDeathRollMax, kBurnDeathTimeMin, kBurnDeathTimeMax);
		REX::INFO("DynamicRagdollSystem: death settings: noDeathAnimation {} chances stumble {} grip {} twitch {}; stiffness {} pushScale {} catchSpeed {} catchRange {} kneeBend {}; dying ends at {} damage, on head loss {}, headless roll x{}; crawl chance {} speed {} interval {} delay {} time {}-{}; twitch delay {} burn roll delay {}"sv,
			kDisableDeathAnimation, kDeathStumbleChance, kDeathGripChance, kTwitchChance, kDeathStiffness, kDeathPushScale, kCatchSpeed, kCatchRange, kDeathKneeBend,
			kDyingEndDamage, kDyingEndOnHeadLoss, kHeadlessRollScale, kCrawlChance, kCrawlSpeed, kCrawlInterval, kCrawlDelay, kCrawlTime.min, kCrawlTime.max, kTwitchDelay, kBurnRollDelay);
		REX::INFO("DynamicRagdollSystem: crawl tuning: pushAccel {} reach {} side {} lift {} elbow {} arm {} maxHeight {} stall {} minPlant {} ankle {} hip {} knee {} headTurn {} headLift {} limbAccel {} limbTorque {} turnTime {}"sv,
			kCrawlAccel, kCrawlReach, kCrawlSide, kCrawlLift, kCrawlElbow, kCrawlArmStrength, kCrawlArmMaxHeight, kCrawlStall, kCrawlMinPlant,
			kCrawlAnkleAccel, kCrawlHipSpeed, kCrawlKneeSpeed, kCrawlHeadTurn, kCrawlHeadLift, kCrawlLimbAccel, kCrawlLimbMaxTorque, kCrawlTurnTime);
		REX::INFO("DynamicRagdollSystem: death ranges: time {}-{} support {}-{} upright {}-{} dying {}-{} twitch {}-{} roll {}-{} (accel {}) braking {}-{}"sv,
			kDeathStumbleTime.min, kDeathStumbleTime.max, kDeathSupport.min, kDeathSupport.max, kDeathUpright.min, kDeathUpright.max,
			kDyingTime.min, kDyingTime.max, kTwitchStrength.min, kTwitchStrength.max, kRollStrength.min, kRollStrength.max, kRollAccel,
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
		if (!GetFileAttributesExA(kSettingsPath, GetFileExInfoStandard, &data)) return;
		const std::uint64_t write = (std::uint64_t{ data.ftLastWriteTime.dwHighDateTime } << 32) | data.ftLastWriteTime.dwLowDateTime;
		if (lastWrite != 0 && write != lastWrite) {
			REX::INFO("DynamicRagdollSystem: settings file changed, reloading"sv);
			LoadSettings();
		}
		lastWrite = write;
	}
}
