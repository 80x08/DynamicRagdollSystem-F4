#include "Common.h"

#include <toml.hpp>

namespace DynamicRagdollSystem
{

	// res/DynamicRagdollSystem.toml, embedded at build time (xmake utils.bin2c); written when the settings file is missing.
	static const char kDefaultSettingsData[] = {
#include "DynamicRagdollSystem.toml.h"
	};
	constexpr std::string_view kDefaultSettings{ kDefaultSettingsData, sizeof(kDefaultSettingsData) - 1 };  // bin2c appends a 0 byte


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


	// sBlacklist: "Plugin.esp|0x800" (FormID inside that plugin, any load order) or "0x000B2BF3" / 0x000B2BF3 (full
	// load-order FormID). Plugin entries resolve once the game data is loaded (LoadSettings runs again on each save load).
	static void ReadBlacklist(const toml::value& a_root, std::string_view a_section, std::vector<RE::TESFormID>& a_list)
	{
		const auto* v = Find(a_root, a_section, "sBlacklist");
		if (!v || !v->is_array()) return;
		auto* data = RE::TESDataHandler::GetSingleton();
		std::vector<RE::TESFormID> ids;
		for (const auto& entry : v->as_array()) {
			RE::TESFormID id = 0;
			std::string text;
			if (entry.is_integer()) {
				id = static_cast<RE::TESFormID>(entry.as_integer());
				text = std::format("{:08X}", id);
			} else if (entry.is_string()) {
				text = entry.as_string();
				const auto bar = text.find('|');
				try {
					id = static_cast<RE::TESFormID>(std::stoul(bar == std::string::npos ? text : text.substr(bar + 1), nullptr, 16));
				} catch (const std::exception&) {
					id = 0;
				}
				if (id && bar != std::string::npos) id = data ? data->LookupFormID(id & 0xFFFFFF, text.substr(0, bar)) : 0;
			}
			if (id) ids.push_back(id);
			else REX::INFO("DynamicRagdollSystem: [{}] sBlacklist: '{}' not resolved (bad entry, or plugin not loaded yet)"sv, a_section, text);
		}
		REX::INFO("DynamicRagdollSystem: [{}] sBlacklist: {} entries"sv, a_section, ids.size());
		std::scoped_lock lock{ g_blacklistMutex };
		a_list = std::move(ids);
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
		ReadBlacklist(root, "Reaction", kReactBlacklist);
		ReadBlacklist(root, "Knockdown", kKnockBlacklist);
		ReadBlacklist(root, "Burning", kBurnBlacklist);
		ReadBlacklist(root, "Death", kDeathBlacklist);
		ReadBlacklist(root, "Crawl", kCrawlBlacklist);
		read("fMinDamage", kMinDamage, 0.0f, 10000.0f);
		read("fCooldown", kCooldown, 0.0f, 60.0f);
		read("fReactionTime", kReactionTime, 0.2f, 10.0f);
		read("fHitSoftness", kHitSoftness, 0.0f, 1.0f);
		read("fPowerArmorHitSoftness", kPowerArmorHitSoftness, 0.0f, 1.0f);
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
			const auto readKnock = [&](const char* a_key, float& a_value, float a_min, float a_max) { ReadFloat(root, "Knockdown", a_key, a_value, a_min, a_max); };
			float hits = static_cast<float>(kMultiHitCount);
			readKnock("iMultiHitCount", hits, 0.0f, 100.0f);
			kMultiHitCount = static_cast<int>(std::lround(hits));
			readKnock("fMultiHitWindow", kMultiHitWindow, 0.001f, 2.0f);
			readKnock("fMultiHitMinDamage", kMultiHitMinDamage, 0.0f, 10000.0f);
			readKnock("fMultiHitChance", kMultiHitChance, 0.0f, 1.0f);
			readKnock("fMultiHitForce", kMultiHitForce, 0.0f, 1000.0f);
			readKnock("fHeavyHitDamage", kHeavyHitDamage, 0.0f, 100000.0f);
			readKnock("fHeavyHitChance", kHeavyHitChance, 0.0f, 1.0f);
			readKnock("fHeavyHitForce", kHeavyHitForce, 0.0f, 1000.0f);
			readKnock("fHitKnockReach", kHitKnockReach, 1.0f, 1000.0f);
			readKnock("fHitKnockMinFraction", kHitKnockMinFraction, 0.0f, 1.0f);
			readKnock("fKnockdownCooldown", kHitKnockCooldown, 0.0f, 600.0f);
			ReadBool(root, "Knockdown", "bKnockdownPowerArmor", kHitKnockPowerArmor);
			ReadBool(root, "Knockdown", "bKnockdownHumanoidOnly", kHitKnockHumanoidOnly);
			ReadBool(root, "Knockdown", "bKnockdownPlayerOnly", kHitKnockPlayerOnly);
			REX::INFO("DynamicRagdollSystem: knockdown settings: burst {} hits in {}s, min damage {}, chance {} force {}; heavy hit {} damage, chance {} force {}; reach {} min fraction {}; cooldown {}s power armor {} humanoid only {} player only {}"sv,
				kMultiHitCount, kMultiHitWindow, kMultiHitMinDamage, kMultiHitChance, kMultiHitForce,
				kHeavyHitDamage, kHeavyHitChance, kHeavyHitForce, kHitKnockReach, kHitKnockMinFraction, kHitKnockCooldown, kHitKnockPowerArmor, kHitKnockHumanoidOnly,
				kHitKnockPlayerOnly);
		}
		{
			const auto readBurn = [&](const char* a_key, float& a_value, float a_min, float a_max) { ReadFloat(root, "Burning", a_key, a_value, a_min, a_max); };
			ReadBool(root, "Burning", "bBurnReaction", kBurnReaction);
			ReadBool(root, "Burning", "bBurnPowerArmor", kBurnPowerArmor);
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
		readDeath("fHeadshotSkipStumbleChance", kHeadshotSkipStumbleChance, 0.0f, 1.0f);
		readDeath("fDeathGripChance", kDeathGripChance, 0.0f, 1.0f);
		readDeath("fTwitchChance", kTwitchChance, 0.0f, 1.0f);
		readRange("fDeathStumbleTime", kDeathStumbleTime, 0.2f, 15.0f);
		readRange("fDyingTime", kDyingTime, 0.0f, 30.0f);
		readRange("fTwitchStrength", kTwitchStrength, 0.0f, 3.0f);
		readRange("fRollStrength", kRollStrength, 0.0f, 3.0f);
		readRange("fPowerArmorTwitchStrength", kPowerArmorTwitchStrength, 0.0f, 3.0f);
		readRange("fPowerArmorRollStrength", kPowerArmorRollStrength, 0.0f, 3.0f);
		readDeath("fRollAccel", kRollAccel, 0.0f, 1000.0f);
		ReadBool(root, "Death", "bDeathRollArms", kDeathRollArms);
		readRange("fFlailStrength", kFlailStrength, 0.0f, 3.0f);
		readDeath("fExplosionStumbleChance", kExplosionStumbleChance, 0.0f, 1.0f);
		readDeath("fDeathKnockbackScale", kDeathKnockbackScale, 0.0f, 1.0f);
		readDeath("fDeathFlingDamage", kDeathFlingDamage, 0.0f, 100000.0f);
		{
			float pellets = static_cast<float>(kDeathFlingPellets);
			readDeath("iDeathFlingPellets", pellets, 0.0f, 100.0f);
			kDeathFlingPellets = static_cast<int>(std::lround(pellets));
		}
		readDeath("fDyingEndDamage", kDyingEndDamage, 0.0f, 10000.0f);
		ReadBool(root, "Death", "bDyingEndOnHeadLoss", kDyingEndOnHeadLoss);
		readDeath("fHeadlessRollScale", kHeadlessRollScale, 0.0f, 1.0f);
		const auto readWalk = [&](const std::string& a_key, float& a_value, float a_min, float a_max) {
			return ReadFloat(root, "DeathWalk", a_key, a_value, a_min, a_max);
		};
		const auto readWalkRange = [&](const std::string& a_key, Range& a_range, float a_min, float a_max) {
			readRangeWith(readWalk, a_key, a_range, a_min, a_max);
		};
		readWalk("fDeathWalkChance", kDeathWalkChance, 0.0f, 1.0f);
		ReadBool(root, "DeathWalk", "bDeathWalkHeadlessOnly", kDeathWalkHeadlessOnly);
		readWalkRange("fDeathWalkTime", kDeathWalkTime, 1.0f, 30.0f);
		readWalk("fDeathWalkPushScale", kDeathWalkPushScale, 0.0f, 5.0f);
		readWalk("fDeathWalkImpactSupport", kDeathWalkImpactSupport, 0.0f, 1.0f);
		readWalk("fDeathWalkImpactTime", kDeathWalkImpactTime, 0.0f, 3.0f);
		readWalk("fDeathWalkSpeed", kDeathWalkSpeed, 0.0f, 3.0f);
		readWalk("fDeathWalkTurnRate", kDeathWalkTurnRate, 0.1f, 10.0f);
		readWalk("fDeathWalkWeave", kDeathWalkWeave, 0.0f, 90.0f);
		readWalkRange("fDeathWalkWeaveTime", kDeathWalkWeaveTime, 0.2f, 10.0f);
		readWalk("fDeathWalkLurch", kDeathWalkLurch, 0.0f, 1.0f);
		readWalk("fDeathWalkDrift", kDeathWalkDrift, 0.0f, 180.0f);
		readWalkRange("fDeathWalkDriftGap", kDeathWalkDriftGap, 0.1f, 10.0f);
		readWalk("fDeathWalkSharpTurnChance", kDeathWalkSharpTurnChance, 0.0f, 1.0f);
		readWalk("fDeathWalkEndDamage", kDeathWalkEndDamage, 0.0f, 10000.0f);
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
		ReadBool(root, "Death", "bRagdollKnockDownPowerArmor", kKnockDownPowerArmor);
		ReadBool(root, "Death", "bRagdollSelfCollision", kSelfCollision);
		readDeath("fRagdollImpactSpeed", kImpactSpeed, 0.0f, 50.0f);
		readDeath("fRagdollImpactForce", kImpactForce, 0.0f, 1000.0f);
		readDeath("fDeathPushScale", kDeathPushScale, 0.0f, 3.0f);
		// [DeathStumble]; these keys used to be in [Death], still read there so older files keep their values.
		const auto readStumble = [&](const std::string& a_key, float& a_value, float a_min, float a_max) {
			readDeath(a_key, a_value, a_min, a_max);
			return ReadFloat(root, "DeathStumble", a_key, a_value, a_min, a_max);
		};
		const auto readStumbleRange = [&](const std::string& a_key, Range& a_range, float a_min, float a_max) {
			readRangeWith(readStumble, a_key, a_range, a_min, a_max);
		};
		readStumble("fDeathStumbleChance", kDeathStumbleChance, 0.0f, 1.0f);
		readStumbleRange("fDeathSupport", kDeathSupport, 0.0f, 1.0f);
		readStumble("fDeathStiffness", kDeathStiffness, 0.0f, 3.0f);
		readStumbleRange("fDeathUpright", kDeathUpright, 0.0f, 3.0f);
		readStumble("fDeathKneeBend", kDeathKneeBend, 0.0f, 120.0f);
		readStumble("fDeathCatchSpeed", kCatchSpeed, 0.0f, 20.0f);
		readStumble("fDeathCatchRange", kCatchRange, 0.0f, 20.0f);
		readStumbleRange("fDeathBraking", kDeathBraking, 0.0f, 20.0f);
		const auto readStagger = [&](const std::string& a_key, float& a_value, float a_min, float a_max) {
			return ReadFloat(root, "DeathStagger", a_key, a_value, a_min, a_max);
		};
		readStagger("fDeathStaggerChance", kDeathStaggerChance, 0.0f, 1.0f);
		readStagger("fDeathStaggerStep", kDeathStaggerStep, 0.0f, 2.0f);
		readRangeWith(readStagger, "fDeathStaggerTip", kDeathStaggerTip, 0.0f, 1.0f);
		readStagger("fDeathStaggerSlump", kDeathStaggerSlump, 0.0f, 5.0f);
		readStagger("fDeathStaggerHurtLeg", kDeathStaggerHurtLeg, 0.0f, 1.0f);
		readStagger("fDeathStaggerFlinch", kDeathStaggerFlinch, 0.0f, 2.0f);
		readStagger("fDeathStaggerGiveUpTilt", kDeathStaggerGiveUpTilt, 0.0f, 3.0f);
		readStagger("fDeathStaggerCatchSpeed", kDeathStaggerCatchSpeed, 0.0f, 20.0f);
		readStagger("fDeathStaggerCatchRange", kDeathStaggerCatchRange, 0.0f, 20.0f);
		readRangeWith(readStagger, "fDeathStaggerBraking", kDeathStaggerBraking, 0.0f, 20.0f);
		readStagger("fDeathStaggerUneven", kDeathStaggerUneven, 0.0f, 3.0f);
		ReadBool(root, "DeathStagger", "bDeathStaggerCrouchStumble", kDeathStaggerCrouchStumble);
		REX::INFO("DynamicRagdollSystem: death stagger: step {} tip {}-{} slump {} hurt leg {} flinch {}s give up {} rad, braking {}-{}"sv,
			kDeathStaggerStep, kDeathStaggerTip.min, kDeathStaggerTip.max, kDeathStaggerSlump, kDeathStaggerHurtLeg, kDeathStaggerFlinch,
			kDeathStaggerGiveUpTilt, kDeathStaggerBraking.min, kDeathStaggerBraking.max);
		REX::INFO("DynamicRagdollSystem: settings: hotReload {}"sv, kIniHotReload);
		REX::INFO("DynamicRagdollSystem: settings: minDamage {} cooldown {} reaction {} softness {} paSoftness {} recover {} blendOut {} push {} bodyPush {} catchUp {} pushDamageRef {} pushScale {}-{} powerAttack {} blocked {}"sv,
			kMinDamage, kCooldown, kReactionTime, kHitSoftness, kPowerArmorHitSoftness, kRecoverTime, kBlendOut, kHitPush, kBodyPush, kMaxCatchUpSpeed,
			kPushDamageRef, kPushScaleMin, kPushScaleMax, kPushPowerAttackMult, kPushBlockedMult);
		REX::INFO("DynamicRagdollSystem: burn settings: reaction {} stiffness {} arms {} flail {} shove {} ({}-{}s) fall {}-{}s force {}; down roll {} ({}-{}s) grace {}s max {}s; death twitch {}-{} roll {}-{} time {}-{}s"sv,
			kBurnReaction, kBurnStiffness, kBurnArmStiffness, kBurnFlail, kBurnShove, kBurnShoveMin, kBurnShoveMax, kBurnFallMin, kBurnFallMax,
			kBurnFallForce, kBurnDownRoll, kBurnRollGapMin, kBurnRollGapMax, kBurnOutGrace, kBurnDownMax, kBurnDeathTwitchMin,
			kBurnDeathTwitchMax, kBurnDeathRollMin, kBurnDeathRollMax, kBurnDeathTimeMin, kBurnDeathTimeMax);
		REX::INFO("DynamicRagdollSystem: death settings: noDeathAnimation {} chances walk {} ({}-{}s) stagger {} stumble {} (head shot skip {}) grip {} twitch {}; stiffness {} pushScale {} catchSpeed {} catchRange {} kneeBend {}; dying ends at {} damage, on head loss {}, headless roll x{}; crawl chance {} speed {} interval {} delay {} time {}-{}; twitch delay {} burn roll delay {}"sv,
			kDisableDeathAnimation, kDeathWalkChance, kDeathWalkTime.min, kDeathWalkTime.max, kDeathStaggerChance, kDeathStumbleChance, kHeadshotSkipStumbleChance, kDeathGripChance, kTwitchChance, kDeathStiffness, kDeathPushScale, kCatchSpeed, kCatchRange, kDeathKneeBend,
			kDyingEndDamage, kDyingEndOnHeadLoss, kHeadlessRollScale, kCrawlChance, kCrawlSpeed, kCrawlInterval, kCrawlDelay, kCrawlTime.min, kCrawlTime.max, kTwitchDelay, kBurnRollDelay);
		REX::INFO("DynamicRagdollSystem: crawl tuning: pushAccel {} reach {} side {} lift {} elbow {} arm {} maxHeight {} stall {} minPlant {} ankle {} hip {} knee {} headTurn {} headLift {} limbAccel {} limbTorque {} turnTime {}"sv,
			kCrawlAccel, kCrawlReach, kCrawlSide, kCrawlLift, kCrawlElbow, kCrawlArmStrength, kCrawlArmMaxHeight, kCrawlStall, kCrawlMinPlant,
			kCrawlAnkleAccel, kCrawlHipSpeed, kCrawlKneeSpeed, kCrawlHeadTurn, kCrawlHeadLift, kCrawlLimbAccel, kCrawlLimbMaxTorque, kCrawlTurnTime);
		REX::INFO("DynamicRagdollSystem: death ranges: time {}-{} support {}-{} upright {}-{} dying {}-{} twitch {}-{} roll {}-{} (accel {}) power armor twitch {}-{} roll {}-{} braking {}-{}; power armor burns {} knocked down {}"sv,
			kDeathStumbleTime.min, kDeathStumbleTime.max, kDeathSupport.min, kDeathSupport.max, kDeathUpright.min, kDeathUpright.max,
			kDyingTime.min, kDyingTime.max, kTwitchStrength.min, kTwitchStrength.max, kRollStrength.min, kRollStrength.max, kRollAccel,
			kPowerArmorTwitchStrength.min, kPowerArmorTwitchStrength.max, kPowerArmorRollStrength.min, kPowerArmorRollStrength.max,
			kDeathBraking.min, kDeathBraking.max, kBurnPowerArmor, kKnockDownPowerArmor);
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
