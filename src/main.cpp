#include "version.h"
#include "Common.h"

using namespace DynamicRagdollSystem;

F4SE_PLUGIN_LOAD(const F4SE::LoadInterface* a_f4se)
{
	F4SE::Init(a_f4se, {
		.log = true,
		.logLevel = DebugLogEnabled() ? REX::ELogLevel::Info : REX::ELogLevel::Warning,
		.logName = "DynamicRagdollSystem",
		.trampoline = true,
		.trampolineSize = 256,
	});

	REX::WARN("DynamicRagdollSystem by 80x08"sv);  // WARN so it shows even when the log level is Warning

	LoadSettings();
	InstallHooks();
	if (const auto* messaging = F4SE::GetMessagingInterface()) {
		messaging->RegisterListener([](F4SE::MessagingInterface::Message* a_msg) {
			if (a_msg && (a_msg->type == F4SE::MessagingInterface::kPreLoadGame || a_msg->type == F4SE::MessagingInterface::kNewGame)) {
				ClearTracking();
			}
			// Settings are re-read whenever a save is loaded (and at game start): edit the INI, then load a save.
			if (a_msg && (a_msg->type == F4SE::MessagingInterface::kPostLoadGame || a_msg->type == F4SE::MessagingInterface::kNewGame)) {
				LoadSettings();
			} else if (a_msg && a_msg->type == F4SE::MessagingInterface::kPreSaveGame) {
				ReleaseAttackLocks();  // never save our kAttackingDisabled; re-set next frame while still burning
			}
		});
	}

	return true;
}

extern "C"
{
	F4SE_EXPORT bool F4SEPlugin_Query(const F4SE::QueryInterface* a_f4se, F4SE::PluginInfo* a_info)
	{
		a_info->name = Version::PROJECT.data();
		a_info->infoVersion = F4SE::PluginInfo::kVersion;
		a_info->version = Version::MAJOR;

		if (a_f4se->IsEditor()) {
			return false;
		}
		return true;
	}
}


