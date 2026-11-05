#include "Common.h"

namespace DynamicRagdollSystem
{

	std::uintptr_t FindRagdollDriver(RE::Actor* a_actor, std::uintptr_t* a_graph)
	{
		if (!a_actor || !a_actor->currentProcess || !a_actor->currentProcess->middleHigh) return 0;
		auto* manager = a_actor->currentProcess->middleHigh->animationGraphManager.get();
		if (!manager || manager->graph.empty() || manager->graph.size() > 8) return 0;
		for (const auto& graphRef : manager->graph) {
			const auto graph = reinterpret_cast<std::uintptr_t>(graphRef.get());
			std::uintptr_t driver = 0, ragdollInterface = 0, driverInterface = 0;
			if (graph && ReadField(graph, 0x208, driver) && ReadField(graph, 0x210, ragdollInterface) && driver &&
				ragdollInterface && ReadField(driver, 0x70, driverInterface) && driverInterface == ragdollInterface) {
				if (a_graph) *a_graph = graph;
				return driver;
			}
		}
		return 0;
	}


	// Direction of a hit (projectile travel, else attacker → target) and the hknp body id it hit, if known.
	void ReadHitInfo(RE::Actor* a_actor, const RE::HitData& a_hit, Vec3& a_dir, std::uint32_t& a_hitBody)
	{
		// DamageImpactData members are private: location +0x00, normal +0x10, velocity +0x20, colObj +0x30.
		const auto* impact = reinterpret_cast<const std::uint8_t*>(&a_hit.impactData);
		Vec3 velocity{};
		std::memcpy(&velocity, impact + 0x20, sizeof(velocity));
		a_dir = Unit(velocity);
		if (a_hit.flags.any(RE::HitData::Flag::kExplosion)) {
			// shortcut: assumes the impact location is the blast point for explosions (not verified in RE);
			// falls back to the attacker direction below when it is missing or on top of the actor.
			Vec3 location{};
			std::memcpy(&location, impact, sizeof(location));
			const auto to = a_actor->GetPosition();
			const Vec3 away{ to.x - location.x, to.y - location.y, 0.0f };
			a_dir = Length(location) > 1.0f && Length(away) > 5.0f ? Unit(away) : Vec3{};
		}
		if (Length(a_dir) < 0.5f) {  // melee/explosion: no projectile velocity, push away from the attacker
			auto from = a_actor->GetPosition();
			if (const auto aggressor = a_hit.aggressor.get(); aggressor && aggressor.get() != a_actor) {
				from = aggressor->GetPosition();
			} else {
				from.x -= std::sin(a_actor->data.angle.z) * 100.0f;
				from.y -= std::cos(a_actor->data.angle.z) * 100.0f;
			}
			const auto to = a_actor->GetPosition();
			a_dir = Unit(Vec3{ to.x - from.x, to.y - from.y, 0.0f });
		}
		a_hitBody = 0x7FFFFFFF;
		RE::bhkNPCollisionObject* colObj = nullptr;
		std::memcpy(&colObj, impact + 0x30, sizeof(colObj));
		if (colObj) {
			RE::hknpBodyId id{};
			a_hitBody = colObj->GetBodyId(id).m_value;
		}
	}


	// Furniture, sitting/sleeping, sync anims (kill moves, paired anims) and interactions: the animation is locked
	// there, and the engine snaps these actors out of any ragdoll. Neither reactions nor death handling touch them.
	// The engine's own check (as 0x1408113D0 does): is the actor in a running sync instance? The sync manager
	// (0x1459D7608) holds instance pointers at +0x18, count at +0x28; 0x14080CE90(instance, &handle, false) is a
	// read-only lookup under the instance lock.
	bool InRunningSync(RE::Actor* a_actor)
	{
		static REL::Relocation<std::uintptr_t*> manager{ REL::ID{ 1346879 } };
		using isIn_t = bool (*)(std::uintptr_t, const std::uint32_t*, bool);
		static REL::Relocation<isIn_t> isIn{ REL::ID{ 1291193 } };
		const auto mgr = *manager.get();
		std::uintptr_t list = 0;
		std::uint32_t count = 0;
		if (!mgr || !ReadField(mgr, 0x18, list) || !ReadField(mgr, 0x28, count)) return true;  // unreadable: stay safe
		if (count > 256) return true;
		const std::uint32_t handle = a_actor->GetHandle().get_handle();
		for (std::uint32_t i = 0; i < count; ++i) {
			std::uintptr_t instance = 0;
			if (ReadField(list, i * sizeof(instance), instance) && instance && isIn(instance, &handle, false)) return true;
		}
		return false;
	}

	// Which locked state the actor is in, or nullptr if none.
	const char* LockedReason(RE::Actor* a_actor)
	{
		const auto* process = a_actor->currentProcess;
		if (process && process->middleHigh && process->middleHigh->occupiedFurniture.get()) return "occupied furniture handle set";
		if (a_actor->DoGetSitSleepState() != RE::SIT_SLEEP_STATE::kNormal) return "sit/sleep state";
		// The sync-anim bit can stay set after a paired animation ends uncleanly (seen on fresh placeatme spawns),
		// so it only counts if the engine's sync manager confirms a running sync for the actor.
		if (a_actor->inSyncAnim) {
			if (InRunningSync(a_actor)) return "in sync anim";
			LogSkipOnce(a_actor, "sync flag", "set but no running sync; ignored");
		}
		if (a_actor->interactingState != static_cast<RE::INTERACTING_STATE>(0)) return "interacting state";
		if (a_actor->boolFlags.all(RE::Actor::BOOL_FLAGS::kIsInKillMove)) return "kill move flag";
		return nullptr;
	}

	bool InLockedAnimation(RE::Actor* a_actor) { return LockedReason(a_actor) != nullptr; }

	// Diagnostic: why an NPC got no reaction, logged once per actor and reason (silent gates otherwise).
	void LogSkipOnce(RE::Actor* a_actor, std::string_view a_where, std::string_view a_why)
	{
		static std::mutex mutex;
		static std::unordered_set<std::string> logged;
		auto line = std::format("{:08X} {}: {}", a_actor->GetFormID(), a_where, a_why);
		std::scoped_lock lock{ mutex };
		if (logged.insert(line).second) REX::INFO("DynamicRagdollSystem: {}"sv, line);
	}

	// Called for every hit before the engine applies it (DoHitMe). Returns true if a reaction was queued.
	bool QueueReaction(RE::Actor* a_actor, const RE::HitData& a_hit)
	{
		if (!a_actor || a_actor->IsPlayerRef() || a_actor->IsDead(false) || a_hit.flags.any(RE::HitData::Flag::kFatal) ||
			a_hit.totalDamage < kMinDamage) {
			return false;
		}
		if (kReactHumanoidOnly && !IsHumanoid(a_actor)) {
			LogSkipOnce(a_actor, "hit skipped", "not humanoid (bHumanoidOnly)");
			return false;
		}
		if (a_actor->lifeState != static_cast<std::uint32_t>(RE::ACTOR_LIFE_STATE::kAlive)) {
			LogSkipOnce(a_actor, "hit skipped", std::format("life state {}", static_cast<std::uint32_t>(a_actor->lifeState)));
			return false;
		}
		if (!a_actor->currentProcess || !a_actor->currentProcess->middleHigh) {
			LogSkipOnce(a_actor, "hit skipped", "no middle-high process");
			return false;
		}
		if (a_actor->knockState != 0) {
			LogSkipOnce(a_actor, "hit skipped", std::format("knock state {}", static_cast<std::uint32_t>(a_actor->knockState)));
			return false;
		}
		if (!GetWorld(a_actor)) {
			LogSkipOnce(a_actor, "hit skipped", "no physics world");
			return false;
		}
		if (const auto why = LockedReason(a_actor)) {
			LogSkipOnce(a_actor, "hit skipped", why);
			return false;
		}

		const float now = NowSeconds();
		std::scoped_lock lock{ g_mutex };
		if (const auto it = g_lastTrigger.find(a_actor->GetFormID()); it != g_lastTrigger.end() && now - it->second < kCooldown) {
			return false;
		}
		g_lastTrigger[a_actor->GetFormID()] = now;

		Vec3 dir{};
		std::uint32_t hitBody = 0x7FFFFFFF;
		ReadHitInfo(a_actor, a_hit, dir, hitBody);
		g_pending.push_back({ a_actor->GetFormID(), dir, hitBody });
		return true;
	}


	void Hook_DoHitMe(RE::Actor* a_actor, RE::HitData* a_hit)
	{
		if (a_hit && a_actor && !a_actor->IsPlayerRef() && !a_actor->IsDead(false)) {
			LastHit last{};
			ReadHitInfo(a_actor, *a_hit, last.dir, last.hitBody);
			last.explosion = a_hit->flags.any(RE::HitData::Flag::kExplosion);
			if (const auto* weapon = a_hit->weapon.object) last.weapon = RE::TESFullName::GetFullName(*weapon);
			last.damage = a_hit->totalDamage;
			last.time = NowSeconds();
			std::scoped_lock lock{ g_mutex };
			g_lastHit[a_actor->GetFormID()] = last;
		} else if (a_hit && a_actor && !a_actor->IsPlayerRef()) {
			// Corpse hit: count it toward finishing off a body in the dying phase (same test as the StepDying dispatch).
			const float now = NowSeconds();
			std::scoped_lock lock{ g_mutex };
			for (auto& death : g_deaths)
				if (death.formID == a_actor->GetFormID() && (now >= death.end || (death.burnDown && !death.joints.empty())))
					death.dyingDamage += a_hit->totalDamage;
		}
		if (a_hit && a_actor && !a_actor->IsPlayerRef()) LogSkipOnce(a_actor, "hit hook", "first hit seen");
		if (a_hit) QueueReaction(a_actor, *a_hit);
		g_doHitMe(a_actor, a_hit);
		if (a_hit && a_actor) {  // diagnostic: the killing hit's final flags/limb are logged when its death starts
			std::scoped_lock lock{ g_mutex };
			if (const auto it = g_lastHit.find(a_actor->GetFormID()); it != g_lastHit.end()) {
				it->second.flags = a_hit->flags.underlying();
				it->second.limb = static_cast<std::int32_t>(a_hit->damageLimb.get());
			}
		}
		// Head dismembered/exploded by this hit (flags read after the engine applied it).
		using Limb = RE::BGSBodyPartDefs::LIMB_ENUM;
		if (a_hit && a_actor && !a_actor->IsPlayerRef() &&
			a_hit->flags.any(RE::HitData::Flag::kDismemberLimb, RE::HitData::Flag::kExplodeLimb)) {
			const auto limb = a_hit->damageLimb.get();
			REX::INFO("DynamicRagdollSystem: {:08X} hit {} limb {}"sv, a_actor->GetFormID(),
				a_hit->flags.any(RE::HitData::Flag::kExplodeLimb) ? "explodes" : "dismembers", static_cast<std::int32_t>(limb));
			if (limb == Limb::kHead1 || limb == Limb::kHead2 || limb == Limb::kEye1 || limb == Limb::kBrain) {
				std::scoped_lock lock{ g_mutex };
				if (g_headless.insert(a_actor->GetFormID()).second)
					REX::INFO("DynamicRagdollSystem: {:08X} head {} by a hit"sv, a_actor->GetFormID(),
						a_hit->flags.any(RE::HitData::Flag::kExplodeLimb) ? "exploded" : "dismembered");
			}
		}
	}


	void InstallHitHook()
	{
		const auto site = REL::ID{ 1546751 }.address() + 0x921;
		const auto target = REL::ID{ 881215 }.address();
		std::uint8_t bytes[5]{};
		std::int32_t rel = 0;
		if (!SafeRead(site, bytes, 5) || bytes[0] != 0xE8 || (std::memcpy(&rel, bytes + 1, 4), site + 5 + rel != target)) {
			REX::INFO("DynamicRagdollSystem: DoHitMe call site mismatch; no hit reactions"sv);
			return;
		}
		g_doHitMe = reinterpret_cast<DoHitMe>(REL::GetTrampoline().write_call<5>(site, &Hook_DoHitMe));
	}


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
		if (rigid->elementSize != sizeof(RigidBodyControl) || rigid->capacity <= 0 || rigid->capacity > 64 || rigid->dataOffset <= 0) {
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
		const auto addEntry = [&](float a_entryStiffness, std::uint64_t a_bones) {
			if (!a_bones || entries >= rigid->capacity) return;
			const auto entry = makeControl(a_entryStiffness);
			std::memcpy(data + static_cast<std::size_t>(entries) * rigid->elementSize, &entry, sizeof(entry));
			for (std::int32_t bone = 0; bone < rigid->capacity && bone < 64; ++bone)
				if (a_bones >> bone & 1) map[bone] = static_cast<std::uint8_t>(entries);
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


	void Hook_DriveToPose(void* a_driver, float a_dt, void* a_context, void* a_output)
	{
		{
			std::scoped_lock lock{ g_driveMutex };
			if (const auto it = g_drives.find(reinterpret_cast<std::uintptr_t>(a_driver)); it != g_drives.end()) {
				auto& state = it->second;
				++state.calls;
				const float now = NowSeconds();
				const float recover = std::clamp((now - state.start) / kRecoverTime, 0.0f, 1.0f);
				const float smooth = recover * recover * (3.0f - 2.0f * recover);
				// During a melee attack (e.g. a full-body machete swing) a very soft ragdoll lags the fast animation
				// and gets wrenched into odd poses; keep it firmer and let it catch up faster.
				const auto* actor = RE::TESForm::GetFormByID<RE::Actor>(state.formID);
				const bool attacking = actor && actor->meleeAttackState != 0;
				const float softness = attacking ? std::max(kHitSoftness, kAttackSoftness) : kHitSoftness;
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
		g_driveToPose(a_driver, a_dt, a_context, a_output);
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
		if (reinterpret_cast<std::uintptr_t>(_ReturnAddress()) == g_graphUpdateReturn) {
			std::uintptr_t driver = 0;
			if (ReadField(reinterpret_cast<std::uintptr_t>(a_graph), 0x208, driver) && driver) {
				std::scoped_lock lock{ g_driveMutex };
				if (g_drives.contains(driver)) return false;
			}
		}
		return (*g_ragdollActiveHook)(a_graph);
	}


	void InstallGraphUpdateHook()
	{
		static constexpr std::uint8_t callSlot19[6] = { 0xFF, 0x90, 0x98, 0x00, 0x00, 0x00 };  // call [rax+0x98]
		const auto call = REL::Offset{ 0x161D9F4 }.address();
		std::uint8_t bytes[6]{};
		std::uintptr_t slot19 = 0;
		if (!SafeRead(call, bytes, sizeof(bytes)) || std::memcmp(bytes, callSlot19, sizeof(bytes)) != 0 ||
			!ReadField(RE::VTABLE::BShkbAnimationGraph[0].address(), 19 * 8, slot19) || slot19 != REL::Offset{ 0x1627BD0 }.address()) {
			REX::INFO("DynamicRagdollSystem: graph update call site mismatch; NPCs may keep their hit-time velocity"sv);
			return;
		}
		g_graphUpdateReturn = call + sizeof(bytes);
		static REL::THookVFT<bool(void*)> hook{ REL::Offset{ RE::VTABLE::BShkbAnimationGraph[0].offset() }, 19, &Hook_IsRagdollActive };
		hook.Init();
		hook.Enable();
		g_ragdollActiveHook = &hook;
	}


	void InstallMovementHook()
	{
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
		const auto site = REL::ID{ 1348502 }.address() + 0x12F;
		const auto target = REL::ID{ 993992 }.address();
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
		const auto site = REL::ID{ 1414229 }.address() + 0x1EA;
		const auto target = REL::ID{ 560275 }.address();
		std::uint8_t bytes[5]{};
		std::int32_t rel = 0;
		if (!SafeRead(site, bytes, 5) || bytes[0] != 0xE8 || (std::memcpy(&rel, bytes + 1, 4), site + 5 + rel != target)) {
			REX::INFO("DynamicRagdollSystem: postPhysics call site mismatch; small skeletons stretch to adult size when ragdolled"sv);
			return;
		}
		g_postPhysics = reinterpret_cast<PostPhysics>(REL::GetTrampoline().write_call<5>(site, &Hook_PostPhysics));
	}


	void ProcessPending()
	{
		std::vector<Pending> pending;
		{
			std::scoped_lock lock{ g_mutex };
			pending.swap(g_pending);
		}

		for (auto& entry : pending) {
			const auto actor = RE::TESForm::GetFormByID<RE::Actor>(entry.formID);
			if (!actor || actor->IsDead(false) || !actor->currentProcess || !actor->currentProcess->middleHigh || !g_driveToPose ||
				IsMismatched(entry.formID)) continue;
			auto* world = GetWorld(actor);
			auto* npWorld = world ? world->m_worldNP.get() : nullptr;
			auto* root = actor->Get3D();
			std::uintptr_t graph = 0;
			const auto driver = FindRagdollDriver(actor, &graph);
			if (!npWorld || !root || !driver) {
				REX::INFO("DynamicRagdollSystem: {:08X} has no ragdoll driver/3D; skipped"sv, entry.formID);
				continue;
			}
			const float now = NowSeconds();
			// Already reacting: restart the reaction with the new hit (sustained fire keeps the NPC reeling).
			{
				std::scoped_lock lock{ g_mutex };
				const auto it = std::find_if(g_stumbles.begin(), g_stumbles.end(),
					[&](const Stumble& a_s) { return a_s.formID == entry.formID; });
				if (it != g_stumbles.end() && entry.burn) {
					it->burning = true;
					it->burnUntil = now + 0.5f;
					continue;
				}
				if (it != g_stumbles.end()) {
					it->start = now;
					it->end = now + kReactionTime;
					it->hitDir = entry.dir;
					it->hitBody = entry.hitBody;
					it->pushed = false;
					std::scoped_lock driveLock{ g_driveMutex };
					if (const auto drive = g_drives.find(driver); drive != g_drives.end()) {
						drive->second.start = now;
						drive->second.end = now + kReactionTime;
					}
					continue;
				}
			}
			{
				WorldWriteLock lock{ npWorld };
				if (!lock.held) continue;  // a later hit retries
				// AddRagdollToWorld (as ForceAddRagdollToWorld does). Per graph it only sets graph+0x3B8 = world,
				// which is what makes the pre-physics update call driveToPose. A live NPC's ragdoll bodies are
				// already in the world, but its graph is not enrolled until this runs. Vanilla never clears it.
				struct AddArgs { RE::bhkWorld* world; bool flag; } args{ world, false };
				using add_t = bool (*)(RE::BSAnimationGraphManager*, AddArgs*);
				static REL::Relocation<add_t> addRagdoll{ REL::ID{ 683498 } };
				auto* manager = actor->currentProcess->middleHigh->animationGraphManager.get();
				addRagdoll(manager, &args);
				// Then what the knockdown does next: graph vtable slot 3 per graph (visitor ID 258136), which
				// re-seeds the ragdoll from the animation and registers it with the world if it is not yet.
				bool registered = false;
				using register_t = bool (*)(RE::BSAnimationGraphManager*, bool*);
				static REL::Relocation<register_t> registerRagdoll{ REL::ID{ 258136 } };
				registerRagdoll(manager, &registered);
				std::uintptr_t graphWorld = 0;
				ReadField(graph, 0x3B8, graphWorld);
				REX::INFO("DynamicRagdollSystem: {:08X} driver {:X} graph world {:X} (expected {:X}) registered {}"sv,
					entry.formID, driver, graphWorld, reinterpret_cast<std::uintptr_t>(world), registered);
				// Bodies still follow the animation here: fit the joints of a skeleton sized unlike its ragdoll. If a
				// joint cannot be fitted, this reaction ends next update and later hits skip the actor.
				if (!FitJointsToSkeleton(actor, npWorld)) {
					std::scoped_lock driveLock{ g_driveMutex };
					g_mismatched.insert(entry.formID);
				}
				// Dynamic bodies on the biped layer collide with the actor's own (now moving) character controller,
				// which then climbs onto them and rises. BipedNoCC still takes projectile hits.
				SetTreeCollisionLayer(root, RE::COL_LAYER::kBipedNoCC);
				SetTreeMotionType(root, RE::hknpMotionPropertiesId::Preset::kDynamic);
			}

			{
				std::scoped_lock lock{ g_driveMutex };
				g_drives[driver] = { entry.formID, now, now + kReactionTime, false,
					reinterpret_cast<std::uintptr_t>(actor->currentProcess->middleHigh->animationGraphManager.get()) };
			}
			{
				std::scoped_lock lock{ g_mutex };
				Stumble stumble{ entry.formID, now, now + kReactionTime };
				stumble.hitDir = entry.dir;
				stumble.hitBody = entry.hitBody;
				stumble.burning = entry.burn;
				stumble.burnUntil = now + 0.5f;
				g_stumbles.push_back(std::move(stumble));
			}
			REX::INFO("DynamicRagdollSystem: {:08X} {} reaction started (hit body {})"sv, entry.formID, entry.burn ? "burning" : "hit",
				entry.hitBody);
		}
	}


	// One-off shove on the upper body, once the ragdoll bodies are dynamic.
	void ApplyHitPush(Stumble& a_stumble, RE::Actor* a_actor, float a_now)
	{
		auto* world = GetWorld(a_actor);
		auto* npWorld = world ? world->m_worldNP.get() : nullptr;
		if (!npWorld || !g_linearImpulse) return;
		WorldWriteLock lock{ npWorld };
		if (!lock.held) return;
		if (a_stumble.bodies.empty()) {
			if (a_stumble.nextScan > a_now) return;
			a_stumble.nextScan = a_now + kScanInterval;
			ScanBodies(a_stumble, a_actor, npWorld);
			if (a_stumble.bodies.empty()) return;
			std::uint64_t feet = 0;
			for (const auto& body : a_stumble.bodies)
				if (body.foot && body.bone >= 0 && body.bone < 64) feet |= std::uint64_t{ 1 } << body.bone;
			std::scoped_lock driveLock{ g_driveMutex };
			for (auto& [driver, state] : g_drives)
				if (state.formID == a_stumble.formID) state.footBones = feet;
		}
		auto& worldBodies = npWorld->m_bodyManager.m_bodies;
		struct Sample { const BodyRef* ref; float inverseMass; Vec3 pos; };
		std::vector<Sample> samples;
		Vec3 com{};
		float mass = 0.0f;
		for (const auto& body : a_stumble.bodies) {
			if (body.index < 0 || body.index >= worldBodies.size() || worldBodies[body.index].m_id.m_value != body.id.m_value) continue;
			MotionState motion{};
			if (!ReadMotionState(npWorld, worldBodies[body.index], motion)) continue;  // not dynamic yet
			samples.push_back({ &body, motion.inverseMass, BodyPosition(worldBodies[body.index]) });
			com = Add(com, Scale(samples.back().pos, 1.0f / motion.inverseMass));
			mass += 1.0f / motion.inverseMass;
		}
		if (samples.size() < a_stumble.bodies.size() / 2 || mass <= 0.0f) return;
		com = Scale(com, 1.0f / mass);
		// The body part that was hit gets kHitPush along the hit; the rest of the upper body kBodyPush.
		// Without a known hit body (melee, explosions) the whole upper body gets kHitPush.
		const bool hitKnown = std::any_of(samples.begin(), samples.end(),
			[&](const Sample& a_s) { return a_s.ref->id.m_value == a_stumble.hitBody; });
		std::int32_t pushed = 0;
		for (const auto& s : samples) {
			const bool isHit = s.ref->id.m_value == a_stumble.hitBody;
			if (!isHit && s.pos.z <= com.z) continue;
			const float speed = a_stumble.burning ? kBurnShove : isHit || !hitKnown ? kHitPush : kBodyPush;  // catching fire: a stagger, not a shot
			auto impulse = Scale(a_stumble.hitDir, speed / s.inverseMass);
			// Feet and calves sit at floor level: a downward shot pushed them through the ground. Horizontal and
			// upward part only (not renormalized, so a steep shot stays a small push).
			if (s.ref->foot || s.ref->knee) impulse.z = std::max(impulse.z, 0.0f);
			RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
			g_linearImpulse(npWorld, s.ref->id.m_value, &nativeImpulse);
			++pushed;
		}
		a_stumble.pushed = true;
		REX::INFO("DynamicRagdollSystem: {:08X} pushed {} bodies (hit body {}) after {:.2f}s"sv,
			a_stumble.formID, pushed, hitKnown ? "found" : "unknown", a_now - a_stumble.start);
	}


	// Keeps the ragdoll on BipedNoCC for the whole reaction. If anything puts a body back on a layer that collides
	// with character controllers (biped 8), the actor's own controller climbs onto its dynamic ragdoll and rises.
	void EnforceNoCCLayer(Stumble& a_stumble, RE::Actor* a_actor)
	{
		auto* world = GetWorld(a_actor);
		auto* npWorld = world ? world->m_worldNP.get() : nullptr;
		auto* root = a_actor->Get3D();
		if (!npWorld || !root || a_stumble.bodies.empty()) return;
		auto& worldBodies = npWorld->m_bodyManager.m_bodies;
		std::uint32_t wrongLayer = 0, foundLayer = 0;
		for (const auto& body : a_stumble.bodies) {
			if (body.index < 0 || body.index >= worldBodies.size() || worldBodies[body.index].m_id.m_value != body.id.m_value) continue;
			const auto layer = worldBodies[body.index].m_collisionFilterInfo & 0x7F;
			if (layer != static_cast<std::uint32_t>(RE::COL_LAYER::kBipedNoCC)) {
				++wrongLayer;
				foundLayer = layer;
			}
		}
		if (!wrongLayer) return;
		WorldWriteLock lock{ npWorld };
		if (!lock.held) return;
		SetTreeCollisionLayer(root, RE::COL_LAYER::kBipedNoCC);
		if (a_stumble.layerResets++ < 3) {
			REX::INFO("DynamicRagdollSystem: {:08X} {} bodies were back on layer {} after {:.2f}s; reset to BipedNoCC"sv,
				a_stumble.formID, wrongLayer, foundLayer, NowSeconds() - a_stumble.start);
		}
	}


	// Diagnostic: is the actor itself moving, and does the ragdoll trunk follow it?
	void TraceMovement(Stumble& a_stumble, RE::Actor* a_actor, float a_now)
	{
		a_stumble.nextTrace = a_now + 0.25f;
		auto* world = GetWorld(a_actor);
		auto* npWorld = world ? world->m_worldNP.get() : nullptr;
		if (!npWorld || a_stumble.bodies.empty()) return;
		const BodyRef* trunk = nullptr;
		for (const auto& body : a_stumble.bodies)
			if (!trunk || body.priority < trunk->priority) trunk = &body;
		auto& worldBodies = npWorld->m_bodyManager.m_bodies;
		if (trunk->index < 0 || trunk->index >= worldBodies.size() || worldBodies[trunk->index].m_id.m_value != trunk->id.m_value) return;
		const auto body = BodyPosition(worldBodies[trunk->index]);
		const auto actorPos = a_actor->GetPosition();
		const auto& rotation = worldBodies[trunk->index].m_transform.m_rotation;
		const float trunkYaw = std::atan2(rotation.m_col[0][1], rotation.m_col[0][0]);
		if (!a_stumble.tracked) {
			a_stumble.tracked = true;
			a_stumble.actorStart = actorPos;
			a_stumble.lastActorPos = actorPos;
			a_stumble.bodyStart = body;
			a_stumble.trunkYawStart = trunkYaw;
			return;
		}
		constexpr float kDegrees = 57.29578f;
		const float stepX = actorPos.x - a_stumble.lastActorPos.x, stepY = actorPos.y - a_stumble.lastActorPos.y;
		a_stumble.lastActorPos = actorPos;
		// Game heading convention: 0 = +Y, clockwise. Travel uses the same convention.
		const float heading = a_actor->data.angle.z * kDegrees;
		const float travel = std::sqrt(stepX * stepX + stepY * stepY) > 2.0f ? std::atan2(stepX, stepY) * kDegrees : NAN;
		float twist = (trunkYaw - a_stumble.trunkYawStart) * kDegrees;
		twist = std::remainder(twist, 360.0f);
		constexpr float kGameUnitsPerPhysicsUnit = 69.99125f;  // 1 / 0.0142875
		const auto bodyMoved = Scale(Sub(body, a_stumble.bodyStart), kGameUnitsPerPhysicsUnit);
		const Vec3 actorMoved{ actorPos.x - a_stumble.actorStart.x, actorPos.y - a_stumble.actorStart.y, actorPos.z - a_stumble.actorStart.z };
		REX::INFO("DynamicRagdollSystem: {:08X} t {:.2f} actor moved ({:.0f},{:.0f},{:.0f}) trunk moved ({:.0f},{:.0f},{:.0f}) heading {:.0f} travel {:.0f} trunk twist {:.0f} deg"sv,
			a_stumble.formID, a_now - a_stumble.start, actorMoved.x, actorMoved.y, actorMoved.z, bodyMoved.x, bodyMoved.y, bodyMoved.z,
			heading, travel, twist);
	}


	bool EndStumble(const Stumble& a_stumble, RE::Actor* a_actor)
	{
		// Back to animation-following bodies, unless the engine took over (death or knockdown ragdoll). Needs the
		// world lock; if it is busy, try again next frame (writing bodies unlocked, or skipping the undo, both break).
		// The legless crawl (kKnockDown) is animation: left dynamic, the ghoul sank through the floor.
		if (a_actor && !a_actor->IsDead(false) && (a_actor->knockState == 0 || a_actor->knockState == kKnockDown)) {
			auto* world = GetWorld(a_actor);
			auto* npWorld = world ? world->m_worldNP.get() : nullptr;
			if (auto* root = a_actor->Get3D(); root && npWorld && a_actor->currentProcess && a_actor->currentProcess->middleHigh) {
				WorldWriteLock lock{ npWorld };
				if (!lock.held) return false;
				SetTreeMotionType(root, RE::hknpMotionPropertiesId::Preset::kKeyframed);
				SetTreeCollisionLayer(root, RE::COL_LAYER::kBiped);
				// Undo the registration (graph slot 3) like the engine's end-of-knockdown does: visitor ID 385024
				// → graph slot 4 → ragdollInterface remove. Without it the NPC stays "ragdolled" and never moves again.
				bool removed = false;
				using remove_t = bool (*)(RE::BSAnimationGraphManager*, bool*);
				static REL::Relocation<remove_t> removeRagdoll{ REL::ID{ 385024 } };
				removeRagdoll(a_actor->currentProcess->middleHigh->animationGraphManager.get(), &removed);
			}
		}
		{
			std::scoped_lock lock{ g_driveMutex };
			for (const auto& [driver, state] : g_drives) {
				if (state.formID == a_stumble.formID) {
					REX::INFO("DynamicRagdollSystem: {:08X} driveToPose calls {} injected {}; movement queries {} overridden {}"sv,
						state.formID, state.calls, state.injected, state.movementQueries, state.movementCleared);
				}
			}
			std::erase_if(g_drives, [&](const auto& a_entry) { return a_entry.second.formID == a_stumble.formID; });
		}
		REX::INFO("DynamicRagdollSystem: {:08X} hit reaction ended"sv, a_stumble.formID);
		return true;
	}

	// On fire: an active effect whose hit shader is one of Fallout4.esm's fire shaders, or with the DamageTypeFire
	// keyword (flamer, Shishkebab, incendiary, Molotov, fire hazards). Never for actors in power armor or effects from
	// abilities. shortcut: base-game forms only; mods that add their own fire shaders/keywords are not detected.
	bool IsBurning(RE::Actor* a_actor, std::string* a_why)
	{
		static constexpr std::array<RE::TESFormID, 12> kFireShaders{ 0x0014237E, 0x001B3F73, 0x0014237D, 0x00145365, 0x00149E5E,
			0x002407FB, 0x00231F8A, 0x00240C20, 0x00211112, 0x00216352, 0x0001D22B, 0x000303B0 };
		constexpr RE::TESFormID kDamageTypeFire = 0x0004A0C8;
		if (RE::PowerArmor::ActorInPowerArmor(*a_actor)) return false;  // power armor: no burning reactions at all
		auto* process = a_actor->currentProcess;
		if (!process || !process->middleHigh) return false;
		for (const auto& entry : process->middleHigh->activeEffects.data) {
			const auto* effect = entry.get();
			if (!effect || !effect->effect || !effect->effect->effectSetting) continue;
			using F = RE::ActiveEffect::Flags;
			if (effect->flags.any(F::kInactive, F::kDispelled, F::kWornOff)) continue;
			const auto* setting = effect->effect->effectSetting;
			// Abilities are never real burning: abGunButtKnockbackSpell (type 4 = ability) has an effect with
			// FireFXS01, and NPCs with it looked like they burned forever. Its effect is fire-and-forget, so the
			// spell type is what tells it apart.
			if (effect->spell && static_cast<std::int32_t>(effect->spell->GetSpellType()) == 4) continue;
			const auto* shader = setting->data.effectShader;
			const bool byKeyword = setting->HasKeywordID(kDamageTypeFire);
			if (byKeyword || (shader && std::find(kFireShaders.begin(), kFireShaders.end(), shader->GetFormID()) != kFireShaders.end())) {
				if (a_why) {
					const auto caster = effect->caster.get();
					*a_why = std::format("effect {:08X} '{}' ({}) from {:08X} '{}', caster {:08X}", setting->GetFormID(),
						RE::TESFullName::GetFullName(*setting), byKeyword ? "DamageTypeFire keyword"s : std::format("fire shader {:08X}", shader->GetFormID()),
						effect->spell ? effect->spell->GetFormID() : 0, effect->spell ? RE::TESFullName::GetFullName(*effect->spell) : ""sv,
						caster ? caster->GetFormID() : 0);
				}
				return true;
			}
		}
		return false;
	}

	// Human, ghoul, feral ghoul or super mutant race (not robots or children): the skeletons the flail is made for.
	bool IsHumanoid(RE::Actor* a_actor)
	{
		const auto* race = a_actor->race;
		if (!race) return false;
		constexpr RE::TESFormID kHuman = 0x0002CB72, kGhoul = 0x000EAFB7, kFeral = 0x0006B4F2, kMutant = 0x0006D7B6;
		constexpr RE::TESFormID kRobot = 0x0002CB73, kChild = 0x001157E8;
		return (race->HasKeywordID(kHuman) || race->HasKeywordID(kGhoul) || race->HasKeywordID(kFeral) ||
		           race->HasKeywordID(kMutant)) &&
		       !race->HasKeywordID(kRobot) && !race->HasKeywordID(kChild);
	}

	// Living NPCs the engine ragdolled (knock state set, not getting up) get a knockdown entry: brace / flail.
	// The burning knockdown already has its own entry (g_deaths) and is left to it.
	void ScanForKnockdowns(float a_now)
	{
		static float nextScan = 0.0f;
		if (a_now < nextScan) return;
		nextScan = a_now + kScanInterval;
		auto* lists = RE::ProcessLists::GetSingleton();
		if (!lists) return;
		static std::mt19937 rng{ std::random_device{}() };
		std::uniform_real_distribution<float> unit{ 0.0f, 1.0f };
		for (const auto& handle : lists->highActorHandles) {
			const auto actor = handle.get();
			if (!actor || actor->IsPlayerRef() || actor->IsDead(false) || actor->knockState == 0 || actor->knockState == kKnockGetUp ||
				actor->knockState == kKnockDown || !actor->Get3D()) continue;
			const auto id = actor->GetFormID();
			std::scoped_lock lock{ g_mutex };
			if (std::any_of(g_knockdowns.begin(), g_knockdowns.end(), [&](const Death& a_d) { return a_d.formID == id; }) ||
				std::any_of(g_deaths.begin(), g_deaths.end(), [&](const Death& a_d) { return a_d.formID == id; })) continue;
			Death down{};
			down.formID = id;
			down.start = a_now;
			down.stumble = false;
			down.grip = false;
			down.flail = kFlailStrength.min + (kFlailStrength.max - kFlailStrength.min) * unit(rng);
			g_knockdowns.push_back(std::move(down));
			REX::INFO("DynamicRagdollSystem: {:08X} knockdown (knock state {}): brace / flail"sv, id, static_cast<std::uint32_t>(actor->knockState));
		}
	}

	// Every 0.25 s: living humanoid NPCs on fire get (or keep) a burning reaction.
	void ScanForBurning(float a_now)
	{
		static float nextScan = 0.0f;
		if (!kBurnReaction || a_now < nextScan) return;
		nextScan = a_now + 0.25f;
		auto* lists = RE::ProcessLists::GetSingleton();
		if (!lists) return;
		static std::mt19937 rng{ std::random_device{}() };
		std::uniform_real_distribution<float> angle{ 0.0f, 6.2831853f };
		for (const auto& handle : lists->highActorHandles) {
			const auto actor = handle.get();
			if (!actor || actor->IsPlayerRef() || actor->IsDead(false) || actor->knockState != 0 || !actor->Get3D()) continue;
			std::string why;
			if (!IsHumanoid(actor.get()) || InLockedAnimation(actor.get()) || !IsBurning(actor.get(), &why)) continue;
			std::scoped_lock lock{ g_mutex };
			if (std::any_of(g_deaths.begin(), g_deaths.end(), [&](const Death& a_d) { return a_d.formID == actor->GetFormID(); })) continue;
			const auto it = std::find_if(g_stumbles.begin(), g_stumbles.end(),
				[&](const Stumble& a_s) { return a_s.formID == actor->GetFormID(); });
			if (it != g_stumbles.end()) {
				if (!it->burning) REX::INFO("DynamicRagdollSystem: {:08X} on fire: {}"sv, actor->GetFormID(), why);
				it->burning = true;
				it->burnUntil = a_now + 0.5f;
				continue;
			}
			if (std::any_of(g_pending.begin(), g_pending.end(), [&](const Pending& a_p) { return a_p.formID == actor->GetFormID(); })) continue;
			REX::INFO("DynamicRagdollSystem: {:08X} on fire: {}"sv, actor->GetFormID(), why);
			const float a = angle(rng);
			g_pending.push_back({ actor->GetFormID(), Vec3{ std::cos(a), std::sin(a), 0.0f }, 0x7FFFFFFF, true });
		}
	}

	// While burning: random stagger shoves on the upper body, and the arms flail (their rigid-body entry is nearly
	// limp, see InjectRigidBodyControls). Joint impulses are paired, so the flail does not move the body as a whole.
	void ApplyBurn(Stumble& a_stumble, RE::Actor* a_actor, float a_now)
	{
		auto* world = GetWorld(a_actor);
		auto* npWorld = world ? world->m_worldNP.get() : nullptr;
		if (!npWorld || !g_linearImpulse || a_stumble.bodies.empty()) return;
		WorldWriteLock lock{ npWorld };
		if (!lock.held) return;
		auto& worldBodies = npWorld->m_bodyManager.m_bodies;
		static std::mt19937 rng{ std::random_device{}() };
		std::uniform_real_distribution<float> unit{ 0.0f, 1.0f };
		if (!a_stumble.burnLogged) {
			a_stumble.burnLogged = true;
			REX::INFO("DynamicRagdollSystem: {:08X} burning: flailing and staggering"sv, a_stumble.formID);
		}

		// Shove: a random horizontal kick on the bodies above the centre of mass.
		if (a_now >= a_stumble.nextShove && kBurnShove > 0.0f) {
			a_stumble.nextShove = a_now + (kBurnShoveMin + (kBurnShoveMax - kBurnShoveMin) * unit(rng)) * (1.0f - 0.5f * a_stumble.burnProgress);
			struct Sample { const BodyRef* ref; float inverseMass; Vec3 pos; };
			std::vector<Sample> samples;
			Vec3 com{};
			float mass = 0.0f;
			for (const auto& body : a_stumble.bodies) {
				if (body.index < 0 || body.index >= worldBodies.size() || worldBodies[body.index].m_id.m_value != body.id.m_value) continue;
				MotionState motion{};
				if (!ReadMotionState(npWorld, worldBodies[body.index], motion)) continue;
				samples.push_back({ &body, motion.inverseMass, BodyPosition(worldBodies[body.index]) });
				com = Add(com, Scale(samples.back().pos, 1.0f / motion.inverseMass));
				mass += 1.0f / motion.inverseMass;
			}
			if (mass > 0.0f) {
				com = Scale(com, 1.0f / mass);
				const float a = unit(rng) * 6.2831853f;
				const Vec3 dir{ std::cos(a), std::sin(a), 0.0f };
				for (const auto& s : samples) {
					if (s.pos.z <= com.z) continue;
					const auto impulse = Scale(dir, kBurnShove * (1.0f + 1.5f * a_stumble.burnProgress) / s.inverseMass);
					RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
					g_linearImpulse(npWorld, s.ref->id.m_value, &nativeImpulse);
				}
			}
		}

		// Arm flail: each arm joint circles about its own random axis (parent frame), like the death flail.
		if (kBurnFlail <= 0.0f) return;
		InitFlail(a_stumble);
		const float t = a_now - a_stumble.start;
		const float dt = 1.0f / 60.0f;  // shortcut: fixed step for the tracking gain, fine for 30-144 fps
		for (std::size_t i = 0; i < a_stumble.joints.size(); ++i) {
			const auto& joint = a_stumble.joints[i];
			if (!joint.arm) continue;
			DriveFlailJoint(npWorld, a_stumble, i, t, kFlailArmSpeed * kBurnFlail, std::min(1.0f, kFlailGain * dt), kFlailMaxTorque * kBurnFlail * dt);
		}
	}

	// Burning NPCs must not shoot: their flailing arms swing the weapon and the shots go anywhere. Actor bool flag
	// kAttackingDisabled (Actor+0x43C bit 20, read by combat code, e.g. 0x1408908D6). Only cleared if we set it.
	void SetAttackLock(Stumble& a_stumble, RE::Actor* a_actor, bool a_lock)
	{
		using F = RE::Actor::BOOL_FLAGS;
		if (a_lock && !a_stumble.attackLock && a_actor && !a_actor->boolFlags.all(F::kAttackingDisabled)) {
			a_actor->boolFlags.set(F::kAttackingDisabled);
			a_stumble.attackLock = true;
			REX::INFO("DynamicRagdollSystem: {:08X} burning: attacks disabled"sv, a_stumble.formID);
		} else if (!a_lock && a_stumble.attackLock) {
			if (a_actor) a_actor->boolFlags.reset(F::kAttackingDisabled);
			a_stumble.attackLock = false;
		}
		// Panic: a temporary Confidence drop makes the combat AI flee instead of turning to face and aim at its
		// target. Temporary modifiers are saved too, so it is undone with the attack lock (and before saves).
		constexpr float kPanic = 4.0f;  // Confidence 0 = cowardly
		auto* values = RE::ActorValue::GetSingleton();
		auto* confidence = values ? values->confidence : nullptr;
		if (!confidence) return;
		if (a_lock && !a_stumble.panicLock && a_actor) {
			a_actor->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *confidence, -kPanic);
			a_stumble.panicLock = true;
		} else if (!a_lock && a_stumble.panicLock) {
			if (a_actor) a_actor->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *confidence, kPanic);
			a_stumble.panicLock = false;
		}
	}

	// Before a save: give back every attack lock so the flag never ends up in a save (re-set next frame if burning).
	void ReleaseAttackLocks()
	{
		std::scoped_lock lock{ g_mutex };
		for (auto& stumble : g_stumbles) {
			if (stumble.attackLock || stumble.panicLock) SetAttackLock(stumble, RE::TESForm::GetFormByID<RE::Actor>(stumble.formID), false);
		}
		for (auto& death : g_deaths) {
			if (death.paralysisLock) SetBurnParalysis(death, RE::TESForm::GetFormByID<RE::Actor>(death.formID), false);
		}
	}

	// Burning NPC on the ground: temporary Paralysis +1 keeps the engine knockdown from ending (see Hooks.cpp).
	// Temporary modifiers are saved, so it is given back before saves and re-applied next frame.
	void SetBurnParalysis(Death& a_death, RE::Actor* a_actor, bool a_lock)
	{
		if (a_lock == a_death.paralysisLock) return;
		auto* values = RE::ActorValue::GetSingleton();
		auto* paralysis = values ? values->paralysis : nullptr;
		if (!paralysis) return;
		if (a_lock && a_actor) {
			a_actor->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *paralysis, 1.0f);
			a_death.paralysisLock = true;
			REX::INFO("DynamicRagdollSystem: {:08X} burning: held down (paralysis)"sv, a_death.formID);
		} else if (!a_lock) {
			if (a_actor) a_actor->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *paralysis, -1.0f);
			a_death.paralysisLock = false;
			REX::INFO("DynamicRagdollSystem: {:08X} burning: released (paralysis)"sv, a_death.formID);
		}
	}

	// Adds a g_deaths entry (g_mutex held), replacing any for the same NPC. A burn-down entry being replaced (the
	// NPC died while held down) gives its Paralysis back first, or the corpse would keep it in the save.
	void ReplaceDeath(Death&& a_death)
	{
		for (auto it = g_deaths.begin(); it != g_deaths.end();) {
			if (it->formID != a_death.formID) {
				++it;
				continue;
			}
			SetBurnParalysis(*it, RE::TESForm::GetFormByID<RE::Actor>(it->formID), false);
			it = g_deaths.erase(it);
		}
		g_deaths.push_back(std::move(a_death));
	}

	// Per frame for a burning reaction: progress towards the fall, attack lock, and the knockdown when time is up.
	void UpdateBurn(Stumble& a_stumble, RE::Actor* a_actor, float a_now)
	{
		static std::mt19937 rng{ std::random_device{}() };
		std::uniform_real_distribution<float> unit{ 0.0f, 1.0f };
		if (a_stumble.burnStart <= 0.0f) {
			a_stumble.burnStart = a_now;
			a_stumble.burnFallTime = kBurnFallMin + (kBurnFallMax - kBurnFallMin) * unit(rng);
		}
		SetAttackLock(a_stumble, a_actor, true);
		a_stumble.burnProgress = a_stumble.burnFallTime > 0.0f ?
		                             std::clamp((a_now - a_stumble.burnStart) / a_stumble.burnFallTime, 0.0f, 1.0f) : 0.0f;
		if (a_stumble.burnFallTime <= 0.0f || a_stumble.burnProgress < 1.0f || a_stumble.burnFell) return;
		// Balance gone. The world update ends this reaction first (undoing our ragdoll registration and drive, which
		// otherwise kept the body pinned to the standing animation), then starts the engine knockdown.
		a_stumble.burnFell = true;
		REX::INFO("DynamicRagdollSystem: {:08X} burning: lost balance after {:.1f}s"sv, a_stumble.formID, a_now - a_stumble.burnStart);
	}

	bool PerformDefaultAction(RE::Actor* a_actor, RE::DEFAULT_OBJECT a_action)
	{
		const auto* manager = RE::BGSDefaultObjectManager::GetSingleton();
		auto* action = manager ? manager->GetDefaultObject<RE::BGSAction>(a_action) : nullptr;
		return action && a_actor->PerformAction(action, nullptr);
	}

	// After the burning reaction ended: the engine knockdown, and a burn-down entry that makes the body writhe
	// and roll on the ground (dying-phase twitches/rolls) while it is still burning.
	void StartBurnDown(RE::Actor* a_actor, float a_now)
	{
		// The engine's explosion knockdown (AIProcess::KnockExplosion → task 10): a real ragdoll pushed away from the
		// given point, then the usual get-up. kActionKnockdown only played a prone animation on living NPCs.
		static std::mt19937 rng{ std::random_device{}() };
		const float angle = std::uniform_real_distribution<float>{ 0.0f, 6.2831853f }(rng);
		auto from = a_actor->GetPosition();
		from.x -= std::cos(angle) * 50.0f;
		from.y -= std::sin(angle) * 50.0f;
		from.z += 50.0f;
		bool done = false;
		if (a_actor->currentProcess) {
			a_actor->currentProcess->KnockExplosion(a_actor, from, kBurnFallForce);
			done = true;
		}
		Death down{};
		down.formID = a_actor->GetFormID();
		down.start = a_now;
		down.end = a_now + 5.0f;  // StepDeath only scans the bodies (no stumble), then the dying phase runs
		down.stumble = false;
		down.grip = false;
		down.hitApplied = true;
		down.twitch = kBurnFlail;
		down.roll = kBurnDownRoll;
		down.dyingTime = kBurnDownMax;
		down.burnDown = true;
		ReplaceDeath(std::move(down));
		REX::INFO("DynamicRagdollSystem: {:08X} burning: knockdown {}"sv, a_actor->GetFormID(), done ? "started" : "failed");
	}

	// bRagdollCollideWithNPCs: fast dead or knocked-down bodies knock over standing NPCs they run into. The bodies now
	// collide with the NPC's limb bodies (SetRagdollNPCCollision) and stop at them, so contact is a distance test of
	// the trunk bodies against the NPC's standing cylinder before the physics step; the victim gets the engine
	// explosion knockdown (as StartBurnDown), pushed along the body's motion.
	void KnockOverStanding(RE::bhkWorld* a_world, float a_now)
	{
		if (!kRagdollCollision || kImpactSpeed <= 0.0f) return;
		auto* npWorld = a_world ? a_world->m_worldNP.get() : nullptr;
		auto* lists = RE::ProcessLists::GetSingleton();
		if (!npWorld || !lists) return;
		constexpr float kGame = 69.99125f;  // game units per physics unit
		constexpr float kRadius = 45.0f;    // game units: the NPC's limbs plus the trunk body's own thickness
		constexpr float kHeight = 130.0f;   // shortcut: fixed human height, upgrade to the actor's bounds if kids/giants matter
		static std::unordered_map<RE::TESFormID, float> knocked;  // victim -> when, so one fall knocks it once
		struct Fast { Vec3 pos; Vec3 vel; };  // trunk body: game units, physics units/s
		std::vector<Fast> fast;
		std::vector<RE::TESFormID> reacting;
		{
			std::scoped_lock lock{ g_mutex };
			const auto collect = [&](const Death& a_d) {
				if (a_d.bodies.empty()) return;
				const auto actor = RE::TESForm::GetFormByID<RE::Actor>(a_d.formID);
				if (!actor || GetWorld(actor) != a_world) return;
				WorldWriteLock worldLock{ npWorld };
				if (!worldLock.held) return;
				auto& worldBodies = npWorld->m_bodyManager.m_bodies;
				for (const auto& body : a_d.bodies) {
					if (!(body.pelvis || body.spine) || body.index < 0 || body.index >= worldBodies.size() ||
						worldBodies[body.index].m_id.m_value != body.id.m_value || a_d.Detached(body.id.m_value)) continue;
					const auto& b = worldBodies[body.index];
					MotionState motion{};
					if (!b.m_flags.all(RE::hknpCollisionFlagsEnum::kIsDynamic) || !ReadMotionState(npWorld, b, motion)) continue;
					const auto& v = motion.linear;
					if (v.x * v.x + v.y * v.y < kImpactSpeed * kImpactSpeed) continue;
					const auto p = BodyPosition(b);
					fast.push_back({ { p.x * kGame, p.y * kGame, p.z * kGame }, v });
				}
			};
			for (const auto& d : g_deaths) collect(d);
			for (const auto& d : g_knockdowns) collect(d);
			if (fast.empty()) return;
			for (const auto& s : g_stumbles) reacting.push_back(s.formID);  // a driven reaction pins the body: skip
		}
		std::erase_if(knocked, [&](const auto& a_e) { return a_now - a_e.second > 3.0f; });
		for (const auto& handle : lists->highActorHandles) {
			const auto actor = handle.get();
			if (!actor || actor->IsPlayerRef() || actor->IsDead(false) || actor->knockState != 0 || !actor->Get3D() ||
				!actor->currentProcess || GetWorld(actor.get()) != a_world) continue;
			const auto id = actor->GetFormID();
			if (knocked.contains(id) || std::find(reacting.begin(), reacting.end(), id) != reacting.end() ||
				!IsHumanoid(actor.get()) || InLockedAnimation(actor.get())) continue;
			const auto pos = actor->GetPosition();
			for (const auto& f : fast) {
				const float dx = pos.x - f.pos.x, dy = pos.y - f.pos.y, dz = f.pos.z - pos.z;
				// inside the cylinder and still moving towards its axis
				if (dx * dx + dy * dy > kRadius * kRadius || dz < 0.0f || dz > kHeight || dx * f.vel.x + dy * f.vel.y <= 0.0f) continue;
				const float speed = std::sqrt(f.vel.x * f.vel.x + f.vel.y * f.vel.y);
				auto from = pos;
				from.x -= f.vel.x / speed * 50.0f;
				from.y -= f.vel.y / speed * 50.0f;
				from.z += 50.0f;
				actor->currentProcess->KnockExplosion(actor.get(), from, kImpactForce);
				knocked[id] = a_now;
				REX::INFO("DynamicRagdollSystem: {:08X} knocked over by a ragdoll at {:.1f} units/s"sv, id, speed);
				break;
			}
		}
	}
}
