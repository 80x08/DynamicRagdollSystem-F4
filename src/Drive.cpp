#include "Common.h"

namespace DynamicRagdollSystem
{
	// Writes rigid-body ragdoll controls into the generator output before driveToPose reads it, so the
	// engine's own keyframe-hierarchy controller pulls the (dynamic) ragdoll toward the animation.
	// Track layout: count at +4, header i at +0x10 + i*0x10 {s16 capacity, s16 numData, s16 dataOffset,
	// s16 elementSize, f32 onFraction, u8 flags, u8 type}; sparse data = palette, then an s8 bone->palette map.
	bool InjectRigidBodyControls(std::uintptr_t a_outputHolder, float a_stiffness, float a_weight, float a_dt, float a_catchUp,
		float a_armStiffness, DriveState& a_state)
	{
		struct Header { std::int16_t capacity, numData, dataOffset, elementSize; float onFraction; std::uint8_t flags, type, pad[2]; };
		std::uintptr_t output = 0;
		std::int32_t   trackCount = 0;
		if (!ReadField(a_outputHolder, 0, output) || !output || !ReadField(output, 4, trackCount) || trackCount < 16) {
			if (!a_state.logged) {
				a_state.logged = true;
				REX::INFO("DynamicRagdollSystem: {:08X} generator output unusable (output {:X}, tracks {})"sv, a_state.formID, output, trackCount);
			}
			return false;
		}
		auto* rigid = reinterpret_cast<Header*>(output + 0x10 + 13 * 0x10);
		auto* powered = reinterpret_cast<Header*>(output + 0x10 + 15 * 0x10);
		if (rigid->elementSize != sizeof(RigidBodyControl) || rigid->capacity <= 0 || rigid->capacity > kMaxRagdollBones || rigid->dataOffset <= 0) {
			// Undrivable (more than kMaxRagdollBones bones): end the reaction now and skip later hits; left dynamic it sank.
			// Caller holds g_driveMutex.
			g_mismatched.insert(a_state.formID);
			if (!a_state.logged) {
				a_state.logged = true;
				REX::INFO("DynamicRagdollSystem: {:08X} unexpected rigid-body track (cap {} elem {:#x}); not driving"sv,
					a_state.formID, rigid->capacity, rigid->elementSize);
			}
			return false;
		}
		const auto makeControl = [&](float a_scale) {
			RigidBodyControl control{};
			// All gains soften on the hit: at full strength the velocity gain cancels the hit push within a few steps.
			control.hierarchyGain *= a_scale;
			control.accelerationGain *= a_scale;
			control.velocityGain *= a_scale;
			control.positionGain *= a_scale;
			control.snapGain *= a_scale;
			// These gains are fractions corrected per update. At high frame rates the engine updates more often and
			// the hit is pulled back much faster, so convert them to the same correction per second as at 30 updates/s.
			const float steps = std::clamp(a_dt * kReferenceRate, 0.05f, 4.0f);
			for (float* gain : { &control.hierarchyGain, &control.velocityGain, &control.positionGain, &control.snapGain }) {
				*gain = 1.0f - std::pow(1.0f - std::clamp(*gain, 0.0f, 0.99f), steps);
			}
			control.positionMaxLinearVelocity = a_catchUp;  // default 1.4 is below running speed
			return control;
		};
		auto* data = reinterpret_cast<std::uint8_t*>(output + rigid->dataOffset);
		const auto mapOffset = (static_cast<std::size_t>(rigid->capacity) * rigid->elementSize + 0xF) & ~std::size_t{ 0xF };
		auto* map = data + mapOffset;
		const auto control = makeControl(a_stiffness);
		std::memcpy(data, &control, sizeof(control));
		std::memset(map, 0, static_cast<std::size_t>(rigid->capacity));  // every bone uses palette entry 0 ...
		std::int16_t entries = 1;
		const auto addEntry = [&](float a_entryStiffness, const BoneMask& a_bones) {
			if (a_bones.none() || entries >= rigid->capacity) return;
			const auto entry = makeControl(a_entryStiffness);
			std::memcpy(data + static_cast<std::size_t>(entries) * rigid->elementSize, &entry, sizeof(entry));
			for (std::int32_t bone = 0; bone < rigid->capacity && bone < kMaxRagdollBones; ++bone)
				if (a_bones[bone]) map[bone] = static_cast<std::uint8_t>(entries);
			++entries;
		};
		// ... except the arms while burning: their own, much softer entry so they can flail.
		if (a_armStiffness >= 0.0f) addEntry(a_armStiffness, a_state.armBones);
		// Feet at full strength: on the soft hit the light foot bodies were pressed into the ground by the body's
		// weight. Not while burning (the burn fall needs loose legs).
		else addEntry(1.0f, a_state.footBones);
		rigid->numData = entries;
		rigid->onFraction = a_weight;
		powered->onFraction = 0.0f;  // rigid-body control only
		// Track 16 = hkbWorldFromModelModeData {s16 bones[3]; s8 mode}. Its default 0 (USE_OLD) froze the
		// actor's root while the ragdoll was driven; USE_INPUT (1) keeps taking it from the animation/controller.
		if (auto* worldFromModel = reinterpret_cast<Header*>(output + 0x10 + 16 * 0x10);
			worldFromModel->elementSize == 8 && worldFromModel->capacity >= 1 && worldFromModel->dataOffset > 0) {
			reinterpret_cast<std::uint8_t*>(output + worldFromModel->dataOffset)[6] = 1;
			worldFromModel->numData = 1;
			worldFromModel->onFraction = 1.0f;
		}
		if (!a_state.logged) {
			a_state.logged = true;
			REX::INFO("DynamicRagdollSystem: {:08X} driving ragdoll (rigid-body track cap {})"sv, a_state.formID, rigid->capacity);
		}
		return true;
	}


	// driveToPose maps the animation onto the ragdoll skeleton (hkaSkeletonMapper::mapPose, mode 2), which ends by
	// giving every parentless bone flagged lockTranslation its reference-pose translation. On AE (1.11.240) humans map
	// onto 'Ragdoll_NPC COM' from skeleton.hkx, whose root has that flag: the target stayed at the standing pelvis
	// height (68.9 up), so crouched NPCs floated. OG maps onto the .nif ragdoll skeleton (root clear). See
	// RE_NOTES_AE.md. Clear it so the root follows the animation. Driver +0x68 -> +0x78 -> +0x30 mapper, +0x18 ragdoll
	// skeleton: parents +0x18 (s16), bones +0x28 (0x10 each, lockTranslation at +8), count +0x30.
	// The skeleton is shared by every human; only this mapping step was checked as a reader of the flag.
	static void UnlockRagdollRoot(std::uintptr_t a_driver, RE::TESFormID a_formID)
	{
		std::uintptr_t a = 0, b = 0, mapper = 0, skeleton = 0, parents = 0, bones = 0;
		std::int32_t   count = 0;
		if (!ReadField(a_driver, 0x68, a) || !a || !ReadField(a, 0x78, b) || !b || !ReadField(b, 0x30, mapper) || !mapper ||
			!ReadField(mapper, 0x18, skeleton) || !skeleton || !ReadField(skeleton, 0x18, parents) || !ReadField(skeleton, 0x28, bones) ||
			!ReadField(skeleton, 0x30, count) || !parents || !bones || count <= 0 || count > kMaxRagdollBones)
			return;
		for (std::int32_t i = 0; i < count; ++i) {
			std::int16_t parent = 0;
			std::uint8_t locked = 0;
			if (!ReadField(parents, i * 2, parent) || parent != -1 || !ReadField(bones, i * 0x10 + 8, locked) || !locked) continue;
			*reinterpret_cast<std::uint8_t*>(bones + i * 0x10 + 8) = 0;
			REX::INFO("DynamicRagdollSystem: {:08X} ragdoll root bone {} had lockTranslation (drive target pinned to the reference pose); cleared"sv,
				a_formID, i);
		}
	}


	void Hook_DriveToPose(void* a_driver, float a_dt, void* a_context, void* a_output)
	{
		float poseScale = 1.0f;
		{
			std::scoped_lock lock{ g_driveMutex };
			if (const auto it = g_drives.find(reinterpret_cast<std::uintptr_t>(a_driver)); it != g_drives.end()) {
				auto& state = it->second;
				if (state.calls++ == 0) UnlockRagdollRoot(reinterpret_cast<std::uintptr_t>(a_driver), state.formID);
				poseScale = state.poseScale;
				const float now = NowSeconds();
				const float recover = std::clamp((now - state.start) / kRecoverTime, 0.0f, 1.0f);
				const float smooth = recover * recover * (3.0f - 2.0f * recover);
				// During a melee attack (e.g. a full-body machete swing) a very soft ragdoll lags the fast animation
				// and gets wrenched into odd poses; keep it firmer and let it catch up faster.
				const auto* actor = RE::TESForm::GetFormByID<RE::Actor>(state.formID);
				const bool attacking = actor && actor->meleeAttackState != 0;
				const float hitSoftness = actor && RE::PowerArmor::ActorInPowerArmor(*actor) ? kPowerArmorHitSoftness : kHitSoftness;
				const float softness = attacking ? std::max(hitSoftness, kAttackSoftness) : hitSoftness;
				const float stiffness = state.burning ? kBurnStiffness * (1.0f - 0.6f * state.burnProgress) :
				                                        softness + (1.0f - softness) * smooth;
				const float catchUp = attacking ? std::max(kMaxCatchUpSpeed, kAttackCatchUpSpeed) : kMaxCatchUpSpeed;
				if (attacking && !state.attackLogged) {
					state.attackLogged = true;
					REX::INFO("DynamicRagdollSystem: {:08X} reaction during a melee attack (state {}): firmer, faster catch-up"sv,
						state.formID, static_cast<std::uint32_t>(actor->meleeAttackState));
				}
				const float weight = std::clamp((state.end - now) / kBlendOut, 0.0f, 1.0f);
				if (a_dt > 0.0f && InjectRigidBodyControls(reinterpret_cast<std::uintptr_t>(a_output), stiffness, weight, a_dt, catchUp,
						state.burning ? kBurnArmStiffness : -1.0f, state)) ++state.injected;
			}
		}
		// driveToPose builds its targets from the local pose (track 2, hkQsTransform 0x30 bytes per bone) without the
		// skeleton root's scale (race height): a 1.72x giant was pulled down to human size, a 0.72x NPC up to it. Scale
		// every bone offset for the call (uniform scale: the whole model-space pose scales about the root), then restore.
		thread_local std::vector<float> saved;
		float*       pose = nullptr;
		std::int32_t count = 0;
		std::uintptr_t output = 0;
		std::int32_t   trackCount = 0;
		if (std::abs(poseScale - 1.0f) > 0.01f && ReadField(reinterpret_cast<std::uintptr_t>(a_output), 0, output) && output &&
			ReadField(output, 4, trackCount) && trackCount > 2 && !(*reinterpret_cast<std::uint8_t*>(output + 0x3C) & 0x10)) {
			count = std::clamp<std::int32_t>(*reinterpret_cast<std::int16_t*>(output + 0x32), 0, 256);
			pose = reinterpret_cast<float*>(output + *reinterpret_cast<std::int16_t*>(output + 0x34));
			saved.resize(static_cast<std::size_t>(count) * 3);
			for (std::int32_t i = 0; i < count; ++i)
				for (std::int32_t k = 0; k < 3; ++k) {
					saved[i * 3 + k] = pose[i * 12 + k];
					pose[i * 12 + k] *= poseScale;
				}
		}
		g_driveToPose(a_driver, a_dt, a_context, a_output);
		for (std::int32_t i = 0; pose && i < count; ++i)
			for (std::int32_t k = 0; k < 3; ++k) pose[i * 12 + k] = saved[i * 3 + k];
	}


	void Hook_MovementRagdollQuery(void* a_manager, char** a_result)
	{
		g_ragdollActiveQuery(a_manager, a_result);
		if (!a_result || !*a_result) return;
		std::scoped_lock lock{ g_driveMutex };
		for (auto& [driver, state] : g_drives) {
			if (state.manager == reinterpret_cast<std::uintptr_t>(a_manager)) {
				++state.movementQueries;
				if (**a_result) {
					++state.movementCleared;
					**a_result = 0;
				}
				return;
			}
		}
	}


	bool Hook_IsRagdollActive(void* a_graph)
	{
		const auto ret = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
		if (ret == g_graphUpdateReturn || ret == g_movementReturn) {
			std::uintptr_t driver = 0;
			if (ReadField(reinterpret_cast<std::uintptr_t>(a_graph), 0x208, driver) && driver) {
				std::scoped_lock lock{ g_driveMutex };
				if (const auto drive = g_drives.find(driver); drive != g_drives.end()) {
					if (ret == g_movementReturn) {  // AE movement query (OG: Hook_MovementRagdollQuery)
						++drive->second.movementQueries;
						if ((*g_ragdollActiveHook)(a_graph)) ++drive->second.movementCleared;
					}
					return false;
				}
			}
		}
		return (*g_ragdollActiveHook)(a_graph);
	}


	// `call [rax+0x98]` (graph vtable slot 19) at a_call; returns the address after it, or 0 on mismatch.
	std::uintptr_t CheckSlot19Call(std::uintptr_t a_call)
	{
		static constexpr std::uint8_t callSlot19[6] = { 0xFF, 0x90, 0x98, 0x00, 0x00, 0x00 };
		std::uint8_t bytes[6]{};
		if (!SafeRead(a_call, bytes, sizeof(bytes)) || std::memcmp(bytes, callSlot19, sizeof(bytes)) != 0) return 0;
		return a_call + sizeof(bytes);
	}


	// Hooks graph vtable slot 19 once; callers set their return address first.
	bool EnableRagdollActiveHook()
	{
		if (g_ragdollActiveHook) return true;
		std::uintptr_t slot19 = 0;
		if (!ReadField(RE::VTABLE::BShkbAnimationGraph[0].address(), 19 * 8, slot19) ||
			slot19 != REL::ID{ 609532, 0, 2256904 }.address()) return false;
		static REL::THookVFT<bool(void*)> hook{ REL::Offset{ RE::VTABLE::BShkbAnimationGraph[0].offset() }, 19, &Hook_IsRagdollActive };
		hook.Init();
		hook.Enable();
		g_ragdollActiveHook = &hook;
		return true;
	}


	void InstallGraphUpdateHook()
	{
		// Graph per-frame update (ID 1273937, OG 0x14161D9A0): its "ragdoll active?" call.
		const auto call = REL::ID{ 1273937, 0, 2256850 }.address() + REL::Offset{ 0x54, 0, 0x69 }.offset();
		const auto ret = CheckSlot19Call(call);
		if (!ret || !EnableRagdollActiveHook()) {
			REX::INFO("DynamicRagdollSystem: graph update call site mismatch; NPCs may keep their hit-time velocity"sv);
			return;
		}
		g_graphUpdateReturn = ret;
	}


	void InstallMovementHook()
	{
		if (REX::FModule::IsRuntimeAE()) {
			// The visitor is inlined into the movement update: its slot-19 call is at + 0x4ED.
			const auto ret = CheckSlot19Call(REL::ID{ 737625, 0, 2229934 }.address() + 0x4ED);
			if (!ret || !EnableRagdollActiveHook()) {
				REX::INFO("DynamicRagdollSystem: movement call site mismatch; NPCs will stop during reactions"sv);
				return;
			}
			g_movementReturn = ret;
			return;
		}
		const auto site = REL::ID{ 737625 }.address() + 0x3B8;
		const auto target = REL::ID{ 453608 }.address();
		std::uint8_t bytes[5]{};
		std::int32_t rel = 0;
		if (!SafeRead(site, bytes, 5) || bytes[0] != 0xE8 || (std::memcpy(&rel, bytes + 1, 4), site + 5 + rel != target)) {
			REX::INFO("DynamicRagdollSystem: movement call site mismatch; NPCs will stop during reactions"sv);
			return;
		}
		g_ragdollActiveQuery = reinterpret_cast<RagdollActiveQuery>(REL::GetTrampoline().write_call<5>(site, &Hook_MovementRagdollQuery));
	}


	void InstallDriveToPoseHook()
	{
		// Call site in the behavior graph pre-physics update: ID 1348502 + 0x12F, `call driveToPose` (ID 993992).
		const auto site = REL::ID{ 1348502, 0, 2256836 }.address() + REL::Offset{ 0x12F, 0, 0x144 }.offset();
		const auto target = REL::ID{ 993992, 0, 2258034 }.address();
		std::uint8_t bytes[5]{};
		std::int32_t rel = 0;
		if (!SafeRead(site, bytes, 5) || bytes[0] != 0xE8 || (std::memcpy(&rel, bytes + 1, 4), site + 5 + rel != target)) {
			REX::INFO("DynamicRagdollSystem: driveToPose call site mismatch; plugin inactive"sv);
			return;
		}
		g_driveToPose = reinterpret_cast<DriveToPose>(REL::GetTrampoline().write_call<5>(site, &Hook_DriveToPose));
	}


	// postPhysics copies the ragdoll pose onto the skeleton. It keeps each bone's scale but takes bone offsets from
	// the ragdoll bodies, whose joints are adult-sized, so a child (or any shrunk/enlarged human skeleton) took on
	// adult proportions in any ragdoll (reaction, knockdown, death). When the offsets disagree, put the animation's
	// back (rotations stay).
	void Hook_PostPhysics(void* a_driver, void* a_context, std::uint32_t a_arg, void* a_output)
	{
		const auto driver = reinterpret_cast<std::uintptr_t>(a_driver);
		// Track 2 = local pose, hkQsTransform {translation, rotation, scale} = 0x30 bytes per bone.
		std::uintptr_t output = 0;
		std::int32_t   trackCount = 0;
		float*         pose = nullptr;
		std::int32_t   count = 0;
		if (ReadField(reinterpret_cast<std::uintptr_t>(a_output), 0, output) && output && ReadField(output, 4, trackCount) &&
			trackCount > 2 && !(*reinterpret_cast<std::uint8_t*>(output + 0x3C) & 0x10)) {
			count = std::clamp<std::int32_t>(*reinterpret_cast<std::int16_t*>(output + 0x32), 0, 256);
			pose = reinterpret_cast<float*>(output + *reinterpret_cast<std::int16_t*>(output + 0x34));
		}
		thread_local std::vector<Vec3> before;
		before.resize(count);
		for (std::int32_t i = 0; i < count; ++i) before[i] = { pose[i * 12], pose[i * 12 + 1], pose[i * 12 + 2] };

		g_postPhysics(a_driver, a_context, a_arg, a_output);
		if (!count) return;

		// Compare only the bones postPhysics rewrote (the ragdoll's ~18): summing the whole skeleton let the
		// untouched fingers/face dilute a child's stretch under the threshold. Bones are parent-first, so the
		// first rewritten one is the ragdoll root; its offset places the body (death/knockdown), so it stays.
		std::int32_t first = -1;
		float animLength = 0.0f, ragdollLength = 0.0f;
		for (std::int32_t i = 0; i < count; ++i) {
			const Vec3 after{ pose[i * 12], pose[i * 12 + 1], pose[i * 12 + 2] };
			const auto d = Sub(after, before[i]);
			if (Dot(d, d) < 1.0e-8f) continue;
			if (first < 0) {
				first = i;
				continue;
			}
			animLength += Length(before[i]);
			ragdollLength += Length(after);
		}
		// Adult skeletons stay within a few percent; ragdoll stretch alone should not trip this. Once a driver trips
		// it stays latched: a child hovers near the threshold, and frames just under it showed the stretch.
		// shortcut: latched per driver address, never cleared; a new actor reusing the address keeps animation bone
		// lengths too (same bone lengths on an adult human, so harmless there).
		static std::unordered_set<std::uintptr_t> latched;
		{
			const bool tripped = animLength > 0.0f && std::abs(ragdollLength / animLength - 1.0f) >= 0.1f;
			std::scoped_lock lock{ g_driveMutex };
			if (!latched.contains(driver)) {
				if (!tripped) return;
				latched.insert(driver);
				REX::INFO("DynamicRagdollSystem: driver {:X} skeleton sized unlike its ragdoll (bone offsets x{:.2f}); keeping animation bone lengths"sv,
					driver, ragdollLength / animLength);
			}
		}
		for (std::int32_t i = first + 1; i < count; ++i) {
			pose[i * 12] = before[i].x;
			pose[i * 12 + 1] = before[i].y;
			pose[i * 12 + 2] = before[i].z;
		}
	}


	void InstallPostPhysicsHook()
	{
		// Graph post-physics update (ID 1414229) + 0x1EA: `call hkbRagdollDriver::postPhysics` (ID 560275).
		const auto site = REL::ID{ 1414229, 0, 2256837 }.address() + REL::Offset{ 0x1EA, 0, 0x1DC }.offset();
		const auto target = REL::ID{ 560275, 0, 2258035 }.address();
		std::uint8_t bytes[5]{};
		std::int32_t rel = 0;
		if (!SafeRead(site, bytes, 5) || bytes[0] != 0xE8 || (std::memcpy(&rel, bytes + 1, 4), site + 5 + rel != target)) {
			REX::INFO("DynamicRagdollSystem: postPhysics call site mismatch; small skeletons stretch to adult size when ragdolled"sv);
			return;
		}
		g_postPhysics = reinterpret_cast<PostPhysics>(REL::GetTrampoline().write_call<5>(site, &Hook_PostPhysics));
	}


}
