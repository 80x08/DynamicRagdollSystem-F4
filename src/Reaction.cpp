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
		static REL::Relocation<std::uintptr_t*> manager{ REL::ID{ 1346879, 0, 4798289 } };
		using isIn_t = bool (*)(std::uintptr_t, const std::uint32_t*, bool);
		static REL::Relocation<isIn_t> isIn{ REL::ID{ 1291193, 0, 2214404 } };
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

	// The actor, its base NPC or race is on a_list, or one of their keywords is (sBlacklist).
	bool IsBlacklisted(RE::Actor* a_actor, const std::vector<RE::TESFormID>& a_list)
	{
		const auto* npc = a_actor->GetNPC();
		const auto* race = a_actor->race;
		std::scoped_lock lock{ g_blacklistMutex };
		return std::any_of(a_list.begin(), a_list.end(), [&](RE::TESFormID a_id) {
			return a_id == a_actor->GetFormID() || (npc && (a_id == npc->GetFormID() || npc->HasKeywordID(a_id))) ||
			       (race && (a_id == race->GetFormID() || race->HasKeywordID(a_id)));
		});
	}

	// Called for every hit before the engine applies it (DoHitMe). Returns true if a reaction was queued.
	bool QueueReaction(RE::Actor* a_actor, const RE::HitData& a_hit)
	{
		if (!a_actor || a_actor->IsPlayerRef() || a_actor->IsDead(false) || a_hit.flags.any(RE::HitData::Flag::kFatal) ||
			a_hit.totalDamage < kMinDamage) {
			return false;
		}
		if (IsBlacklisted(a_actor, kReactBlacklist)) {
			LogSkipOnce(a_actor, "hit skipped", "[Reaction] sBlacklist");
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
			auto& entry = g_lastHit[a_actor->GetFormID()];
			// Pellet burst: hits within fMultiHitWindow of the first are counted (iDeathFlingPellets).
			const bool sameBurst = last.time - entry.burstStart <= kMultiHitWindow;
			last.burstStart = sameBurst ? entry.burstStart : last.time;
			last.burstHits = (sameBurst ? entry.burstHits : 0) + 1;
			entry = last;
		} else if (a_hit && a_actor && !a_actor->IsPlayerRef()) {
			// Corpse hit: count it toward finishing off a body in the dying phase (same test as the StepWrithe dispatch).
			const float now = NowSeconds();
			std::scoped_lock lock{ g_mutex };
			// Pellets of the killing burst that land just after the fatal one, before the death is picked up.
			if (const auto it = g_lastHit.find(a_actor->GetFormID()); it != g_lastHit.end() && now - it->second.burstStart <= kMultiHitWindow)
				++it->second.burstHits;
			for (auto& death : g_deaths) {
				if (death.formID != a_actor->GetFormID()) continue;
				// A walking body keeps its own count (kDeathWalkEndDamage), after the killing burst's own pellets.
				if (death.walk && now < death.end) {
					if (now - death.start > kMultiHitWindow) death.walkDamage += a_hit->totalDamage;
				} else if (now >= death.end || (death.burnDown && !death.joints.empty()))
					death.dyingDamage += a_hit->totalDamage;
			}
		}
		if (a_hit && a_actor && !a_actor->IsPlayerRef()) LogSkipOnce(a_actor, "hit hook", "first hit seen");
		const bool queued = a_hit && QueueReaction(a_actor, *a_hit);
		g_doHitMe(a_actor, a_hit);
		if (a_hit && a_actor) {  // diagnostic: the killing hit's final flags/limb are logged when its death starts
			// The push scale needs the flags the engine sets while applying the hit (blocked).
			const float pushScale = HitPushScale(*a_hit);
			std::scoped_lock lock{ g_mutex };
			if (const auto it = g_lastHit.find(a_actor->GetFormID()); it != g_lastHit.end()) {
				it->second.flags = a_hit->flags.underlying();
				it->second.limb = static_cast<std::int32_t>(a_hit->damageLimb.get());
				it->second.pushScale = pushScale;
			}
			if (queued)
				for (auto& entry : g_pending | std::views::reverse)
					if (entry.formID == a_actor->GetFormID()) {
						entry.pushScale = pushScale;
						break;
					}
		}
		if (a_hit && a_actor) CountKnockdownHit(a_actor, *a_hit);  // Knockdown.cpp
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
		const auto site = REL::ID{ 1546751, 0, 2229323 }.address() + REL::Offset{ 0x921, 0, 0x8F7 }.offset();
		const auto target = REL::ID{ 881215, 0, 2231148 }.address();
		std::uint8_t bytes[5]{};
		std::int32_t rel = 0;
		if (!SafeRead(site, bytes, 5) || bytes[0] != 0xE8 || (std::memcpy(&rel, bytes + 1, 4), site + 5 + rel != target)) {
			REX::INFO("DynamicRagdollSystem: DoHitMe call site mismatch; no hit reactions"sv);
			return;
		}
		g_doHitMe = reinterpret_cast<DoHitMe>(REL::GetTrampoline().write_call<5>(site, &Hook_DoHitMe));
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
					it->pushScale = entry.pushScale;
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
				using add_t = bool (*)(void*, AddArgs*);
				static REL::Relocation<add_t> addRagdoll{ REL::ID{ 683498, 0, 2230588 } };
				auto* target = GraphVisitorTarget(actor);
				addRagdoll(target, &args);
				// Then what the knockdown does next: graph vtable slot 3 per graph (visitor ID 258136), which
				// re-seeds the ragdoll from the animation and registers it with the world if it is not yet.
				bool registered = false;
				using register_t = bool (*)(void*, bool*);
				static REL::Relocation<register_t> registerRagdoll{ REL::ID{ 258136, 0, 2230583 } };
				registerRagdoll(target, &registered);
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
				g_drives[driver].poseScale = root->world.scale;
				if (std::abs(root->world.scale - 1.0f) > 0.01f)
					REX::INFO("DynamicRagdollSystem: {:08X} skeleton scale {:.3f}: drive targets scaled"sv, entry.formID, root->world.scale);
			}
			{
				std::scoped_lock lock{ g_mutex };
				Stumble stumble{ entry.formID, now, now + kReactionTime };
				stumble.hitDir = entry.dir;
				stumble.hitBody = entry.hitBody;
				stumble.pushScale = entry.pushScale;
				stumble.burning = entry.burn;
				stumble.burnUntil = now + 0.5f;
				g_stumbles.push_back(std::move(stumble));
			}
			REX::INFO("DynamicRagdollSystem: {:08X} {} reaction started (hit body {})"sv, entry.formID, entry.burn ? "burning" : "hit",
				entry.hitBody);
		}
	}


	// Push scale from the hit: damage relative to fPushDamageRef (clamped), times the power-attack/blocked factors.
	float HitPushScale(const RE::HitData& a_hit)
	{
		float scale = kPushDamageRef > 0.0f ? std::clamp(a_hit.totalDamage / kPushDamageRef, kPushScaleMin, kPushScaleMax) : 1.0f;
		if (a_hit.flags.any(RE::HitData::Flag::kPowerAttack)) scale *= kPushPowerAttackMult;
		if (a_hit.flags.any(RE::HitData::Flag::kBlocked)) scale *= kPushBlockedMult;
		return scale;
	}

	// Joints between a body and the hit body along the ragdoll tree, or -1 if more than kPushRings (or not linked).
	static std::int32_t JointsFromHit(const std::vector<BodyRef>& a_bodies, std::uint32_t a_hit, std::uint32_t a_id)
	{
		std::vector<std::uint32_t> ring{ a_hit }, seen{ a_hit };
		const auto has = [](const std::vector<std::uint32_t>& a_list, std::uint32_t a_v) { return std::find(a_list.begin(), a_list.end(), a_v) != a_list.end(); };
		for (std::int32_t joints = 0; joints <= kPushRings && !ring.empty(); ++joints) {
			if (has(ring, a_id)) return joints;
			std::vector<std::uint32_t> next;
			for (const auto& body : a_bodies) {
				const auto id = body.id.m_value, parent = body.parentId.m_value;
				if (has(ring, parent) && !has(seen, id)) next.push_back(id), seen.push_back(id);
				if (has(ring, id) && !has(seen, parent)) next.push_back(parent), seen.push_back(parent);
			}
			ring.swap(next);
		}
		return -1;
	}

	// Speed of the hit push for one body (physics units/s; 0 = not pushed): the hit body gets fHitPush, bodies 1..3
	// joints from it fBodyPush halved per further joint. Unknown hit body (melee, explosions): the whole upper body
	// (above the centre of mass) gets fHitPush. Scaled by the hit's damage/flags.
	float PushSpeed(const Stumble& a_stumble, const BodyRef& a_body, bool a_hitKnown, float a_bodyZ, float a_comZ)
	{
		float speed = 0.0f;
		if (!a_hitKnown) {
			speed = a_bodyZ > a_comZ ? kHitPush : 0.0f;
		} else if (const auto joints = JointsFromHit(a_stumble.bodies, a_stumble.hitBody, a_body.id.m_value); joints == 0) {
			speed = kHitPush;
		} else if (joints > 0) {
			speed = kBodyPush * std::ldexp(1.0f, 1 - joints);
		}
		return speed * a_stumble.pushScale;
	}

	// One-off shove, once the ragdoll bodies are dynamic.
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
		}
		auto& worldBodies = npWorld->m_bodyManager.m_bodies;
		struct Sample { const BodyRef* ref; float inverseMass; Vec3 pos; };
		std::vector<Sample> samples;
		Vec3 com{};
		float mass = 0.0f;
		float lowest = std::numeric_limits<float>::max();
		for (const auto& body : a_stumble.bodies) {
			if (!FindBody(worldBodies, body)) continue;
			MotionState motion{};
			if (!ReadMotionState(npWorld, worldBodies[body.index], motion)) continue;  // not dynamic yet
			samples.push_back({ &body, motion.inverseMass, BodyPosition(worldBodies[body.index]) });
			com = Add(com, Scale(samples.back().pos, 1.0f / motion.inverseMass));
			mass += 1.0f / motion.inverseMass;
			lowest = std::min(lowest, samples.back().pos.z);
		}
		if (samples.size() < a_stumble.bodies.size() / 2 || mass <= 0.0f) return;
		com = Scale(com, 1.0f / mass);
		// Floor level by height above the lowest body, not bone names: creature feet are Ankle/Toe/LFOOT.
		// shortcut: fixed heights tuned for human-sized skeletons; scale by the actor's height if giants misbehave.
		BoneMask feet;
		for (const auto& s : samples)
			if (s.pos.z - lowest < kFootBodyHeight && s.ref->bone >= 0 && s.ref->bone < kMaxRagdollBones) feet.set(s.ref->bone);
		{
			std::scoped_lock driveLock{ g_driveMutex };
			for (auto& [driver, state] : g_drives)
				if (state.formID == a_stumble.formID) state.footBones = feet;
		}
		const bool hitKnown = std::any_of(samples.begin(), samples.end(),
			[&](const Sample& a_s) { return a_s.ref->id.m_value == a_stumble.hitBody; });
		std::int32_t pushed = 0;
		for (const auto& s : samples) {
			const float speed = a_stumble.burning ? (s.pos.z > com.z ? kBurnShove : 0.0f) :  // catching fire: a stagger, not a shot
			                                        PushSpeed(a_stumble, *s.ref, hitKnown, s.pos.z, com.z);
			if (speed <= 0.0f) continue;
			auto impulse = Scale(a_stumble.hitDir, speed / s.inverseMass);
			// Floor-level bodies: a downward shot pushed them through the ground. Horizontal and upward part only
			// (not renormalized, so a steep shot stays a small push).
			if (s.pos.z - lowest < kFloorBodyHeight) impulse.z = std::max(impulse.z, 0.0f);
			RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
			g_linearImpulse(npWorld, s.ref->id.m_value, &nativeImpulse);
			++pushed;
		}
		a_stumble.pushed = true;
		REX::INFO("DynamicRagdollSystem: {:08X} pushed {} bodies x{:.2f} (hit body {}) after {:.2f}s"sv,
			a_stumble.formID, pushed, a_stumble.pushScale, hitKnown ? "found" : "unknown", a_now - a_stumble.start);
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
			if (!FindBody(worldBodies, body)) continue;
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
		if (!FindBody(worldBodies, *trunk)) return;
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
		// A downed essential (IsReallyDead) is animated too: left dynamic, it fell through the floor.
		if (a_actor && !IsReallyDead(a_actor) && (a_actor->knockState == 0 || a_actor->knockState == kKnockDown)) {
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
				using remove_t = bool (*)(void*, bool*);
				static REL::Relocation<remove_t> removeRagdoll{ REL::ID{ 385024, 0, 2230587 } };
				removeRagdoll(GraphVisitorTarget(a_actor), &removed);
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

	// IsDead(false) is also true for a downed essential NPC (companion waiting for a stimpak: lifeState 7, an animated
	// bleedout pose). That one is alive: its ragdoll must go back to the animation, and it gets no death handling.
	bool IsReallyDead(RE::Actor* a_actor)
	{
		using State = RE::ACTOR_LIFE_STATE;
		const auto state = a_actor->lifeState;
		return a_actor->IsDead(false) && !a_actor->boolFlags.all(RE::Actor::BOOL_FLAGS::kEssential) &&
		       state != static_cast<std::uint32_t>(State::kEssentialDown) && state != static_cast<std::uint32_t>(State::kBleedout);
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

	bool PerformDefaultAction(RE::Actor* a_actor, RE::DEFAULT_OBJECT a_action)
	{
		const auto* manager = RE::BGSDefaultObjectManager::GetSingleton();
		auto* action = manager ? manager->GetDefaultObject<RE::BGSAction>(a_action) : nullptr;
		return action && a_actor->PerformAction(action, nullptr);
	}
}
