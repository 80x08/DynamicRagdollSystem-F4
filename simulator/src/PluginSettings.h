#pragma once

// The plugin's own settings the death mechanics use, read from its file (../res/DynamicRagdollSystem.toml) with the
// plugin's key names and ranges (../src/Settings.cpp). Defaults are the plugin's (../src/Common.h); a missing key keeps
// its default. Ranges are [0] = fXMin, [1] = fXMax.

#include "ConfigIO.h"

#include <utility>

struct PluginSettings
{
	float hitPush = 8.0f, bodyPush = 5.5f;  // [Reaction]
	// Which death a left click rolls (the plugin's StartPendingDeaths): [Death] head shot skip and grip, then walk (only
	// headless if walkHeadlessOnly; the sim never dismembers), stagger, stumble.
	float headshotSkipChance = 0.1f, gripChance = 0.75f, walkChance = 0.3f, staggerChance = 1.0f, stumbleChance = 1.0f;
	bool  walkHeadlessOnly = true;
	// [Death]
	float stumbleTime[2]{ 2.3f, 3.0f };
	float deathPushScale = 0.4f, deathBrace = 2.5f;
	// [DeathStumble]
	float support[2]{ 0.6f, 0.8f }, upright[2]{ 0.4f, 0.6f }, braking[2]{ 2.4f, 2.7f };
	float stiffness = 0.2f, kneeBend = 80.0f, catchSpeed = 4.0f, catchRange = 6.0f;
	// [DeathStagger]
	float staggerStep = 1.0f, staggerTip[2]{ 0.15f, 0.25f }, staggerSlump = 2.0f, staggerHurtLeg = 0.45f, staggerFlinch = 0.35f;
	float staggerGiveUpTilt = 1.5f, staggerCatchSpeed = 8.0f, staggerCatchRange = 12.0f, staggerBraking[2]{ 0.2f, 1.0f };
	// [DeathWalk]
	float walkTime[2]{ 4.0f, 8.0f }, walkPushScale = 0.3f, walkImpactSupport = 0.9f, walkImpactTime = 0.9f, walkSpeed = 1.3f;
	float walkTurnRate = 2.5f, walkWeave = 45.0f, walkWeaveTime[2]{ 0.9f, 1.8f }, walkLurch = 0.5f, walkDrift = 60.0f;
	float walkDriftGap[2]{ 0.8f, 2.0f }, walkSharpTurnChance = 0.3f;
};
inline PluginSettings g_plugin;

inline std::vector<Lab::Setting> PluginTable(PluginSettings& p)
{
	std::vector<Lab::Setting> t{
		{ "Reaction", "fHitPush", &p.hitPush, 0, 50, "" },
		{ "Reaction", "fBodyPush", &p.bodyPush, 0, 50, "" },
		{ "Death", "fHeadshotSkipStumbleChance", &p.headshotSkipChance, 0, 1, "" },
		{ "Death", "fDeathGripChance", &p.gripChance, 0, 1, "" },
		{ "Death", "fDeathStumbleTimeMin", &p.stumbleTime[0], 0.2f, 15, "" },
		{ "Death", "fDeathStumbleTimeMax", &p.stumbleTime[1], 0.2f, 15, "" },
		{ "Death", "fDeathPushScale", &p.deathPushScale, 0, 3, "" },
		{ "Death", "fDeathBrace", &p.deathBrace, 0, 3, "" },
		{ "DeathStumble", "fDeathStumbleChance", &p.stumbleChance, 0, 1, "" },
		{ "DeathStumble", "fDeathSupportMin", &p.support[0], 0, 1, "" },
		{ "DeathStumble", "fDeathSupportMax", &p.support[1], 0, 1, "" },
		{ "DeathStumble", "fDeathStiffness", &p.stiffness, 0, 3, "" },
		{ "DeathStumble", "fDeathUprightMin", &p.upright[0], 0, 3, "" },
		{ "DeathStumble", "fDeathUprightMax", &p.upright[1], 0, 3, "" },
		{ "DeathStumble", "fDeathKneeBend", &p.kneeBend, 0, 120, "" },
		{ "DeathStumble", "fDeathCatchSpeed", &p.catchSpeed, 0, 20, "" },
		{ "DeathStumble", "fDeathCatchRange", &p.catchRange, 0, 20, "" },
		{ "DeathStumble", "fDeathBrakingMin", &p.braking[0], 0, 20, "" },
		{ "DeathStumble", "fDeathBrakingMax", &p.braking[1], 0, 20, "" },
		{ "DeathStagger", "fDeathStaggerChance", &p.staggerChance, 0, 1, "" },
		{ "DeathStagger", "fDeathStaggerStep", &p.staggerStep, 0, 2, "" },
		{ "DeathStagger", "fDeathStaggerTipMin", &p.staggerTip[0], 0, 1, "" },
		{ "DeathStagger", "fDeathStaggerTipMax", &p.staggerTip[1], 0, 1, "" },
		{ "DeathStagger", "fDeathStaggerSlump", &p.staggerSlump, 0, 5, "" },
		{ "DeathStagger", "fDeathStaggerHurtLeg", &p.staggerHurtLeg, 0, 1, "" },
		{ "DeathStagger", "fDeathStaggerFlinch", &p.staggerFlinch, 0, 2, "" },
		{ "DeathStagger", "fDeathStaggerGiveUpTilt", &p.staggerGiveUpTilt, 0, 3, "" },
		{ "DeathStagger", "fDeathStaggerCatchSpeed", &p.staggerCatchSpeed, 0, 20, "" },
		{ "DeathStagger", "fDeathStaggerCatchRange", &p.staggerCatchRange, 0, 20, "" },
		{ "DeathStagger", "fDeathStaggerBrakingMin", &p.staggerBraking[0], 0, 20, "" },
		{ "DeathStagger", "fDeathStaggerBrakingMax", &p.staggerBraking[1], 0, 20, "" },
		{ "DeathWalk", "fDeathWalkChance", &p.walkChance, 0, 1, "" },
		{ "DeathWalk", "bDeathWalkHeadlessOnly", nullptr, 0, 1, "", &p.walkHeadlessOnly },
		{ "DeathWalk", "fDeathWalkTimeMin", &p.walkTime[0], 1, 30, "" },
		{ "DeathWalk", "fDeathWalkTimeMax", &p.walkTime[1], 1, 30, "" },
		{ "DeathWalk", "fDeathWalkPushScale", &p.walkPushScale, 0, 5, "" },
		{ "DeathWalk", "fDeathWalkImpactSupport", &p.walkImpactSupport, 0, 1, "" },
		{ "DeathWalk", "fDeathWalkImpactTime", &p.walkImpactTime, 0, 3, "" },
		{ "DeathWalk", "fDeathWalkSpeed", &p.walkSpeed, 0, 3, "" },
		{ "DeathWalk", "fDeathWalkTurnRate", &p.walkTurnRate, 0.1f, 10, "" },
		{ "DeathWalk", "fDeathWalkWeave", &p.walkWeave, 0, 90, "" },
		{ "DeathWalk", "fDeathWalkWeaveTimeMin", &p.walkWeaveTime[0], 0.2f, 10, "" },
		{ "DeathWalk", "fDeathWalkWeaveTimeMax", &p.walkWeaveTime[1], 0.2f, 10, "" },
		{ "DeathWalk", "fDeathWalkLurch", &p.walkLurch, 0, 1, "" },
		{ "DeathWalk", "fDeathWalkDrift", &p.walkDrift, 0, 180, "" },
		{ "DeathWalk", "fDeathWalkDriftGapMin", &p.walkDriftGap[0], 0.1f, 10, "" },
		{ "DeathWalk", "fDeathWalkDriftGapMax", &p.walkDriftGap[1], 0.1f, 10, "" },
		{ "DeathWalk", "fDeathWalkSharpTurnChance", &p.walkSharpTurnChance, 0, 1, "" },
	};
	return t;
}

// After reading: a range written the wrong way round is swapped, as the plugin does.
inline void FixRanges(PluginSettings& p)
{
	for (float* r : { p.stumbleTime, p.support, p.upright, p.braking, p.staggerTip, p.staggerBraking, p.walkTime, p.walkWeaveTime, p.walkDriftGap })
		if (r[1] < r[0]) std::swap(r[0], r[1]);
}
