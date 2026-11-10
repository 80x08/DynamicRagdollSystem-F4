#include "Common.h"

namespace DynamicRagdollSystem
{
	// [Knockdown] Living NPCs knocked over by bullet hits: a burst (shotgun pellets land within a few ms, each its own
	// hit) or one heavy hit. Explosion hits never count: explosions knock NPCs down by themselves. Hits are counted in
	// the hit hook after the engine applied them; the knockdown starts next update.

	static bool Roll(float a_chance)
	{
		return Random() < a_chance;
	}

	// g_mutex held. Starts the cooldown and drops a not-yet-started reaction to the same hits.
	static void QueueKnock(RE::TESFormID a_id, Vec3 a_dir, float a_force, float a_now)
	{
		g_lastKnock[a_id] = a_now;
		g_pendingKnocks.push_back({ a_id, Unit(a_dir), a_force });
		std::erase_if(g_pending, [&](const Pending& a_p) { return a_p.formID == a_id && !a_p.burn; });
	}

	// Pellets that land on a corpse, and the killing hit itself, are not counted.
	void CountKnockdownHit(RE::Actor* a_actor, const RE::HitData& a_hit)
	{
		if ((kMultiHitCount <= 0 && kHeavyHitDamage <= 0.0f) || a_actor->IsPlayerRef() || a_actor->IsDead(false) ||
			a_hit.totalDamage <= 0.0f || a_hit.flags.any(RE::HitData::Flag::kFatal, RE::HitData::Flag::kExplosion)) return;
		if (kHitKnockPlayerOnly) {
			const auto aggressor = a_hit.aggressor.get();
			if (!aggressor || !aggressor->IsPlayerRef()) return;
		}
		if (IsBlacklisted(a_actor, kKnockBlacklist) || (kHitKnockHumanoidOnly && !IsHumanoid(a_actor)) ||
			(!kHitKnockPowerArmor && RE::PowerArmor::ActorInPowerArmor(*a_actor))) return;
		Vec3 dir{};
		std::uint32_t hitBody = 0;
		ReadHitInfo(a_actor, a_hit, dir, hitBody);
		const auto id = a_actor->GetFormID();
		const float now = NowSeconds();
		std::scoped_lock lock{ g_mutex };
		const auto cooling = [&] {
			const auto last = g_lastKnock.find(id);
			return last != g_lastKnock.end() && now - last->second < kHitKnockCooldown;
		};

		if (kHeavyHitDamage > 0.0f && a_hit.totalDamage >= kHeavyHitDamage && !cooling()) {
			const bool knock = Roll(kHeavyHitChance);
			REX::INFO("DynamicRagdollSystem: {:08X} heavy hit: {:.1f} damage; knockdown {}"sv, id, a_hit.totalDamage, knock ? "queued" : "not rolled");
			if (knock) QueueKnock(id, dir, kHeavyHitForce, now);
		}

		if (kMultiHitCount <= 0) return;
		auto& burst = g_hitBursts[id];
		// shortcut: fixed window from the burst's first hit, not sliding; fine for pellets, which land within a few ms.
		if (now - burst.start > kMultiHitWindow) burst = HitBurst{ now };
		++burst.hits;
		burst.damage += a_hit.totalDamage;
		burst.dir = Add(burst.dir, dir);
		if (burst.rolled || burst.hits < kMultiHitCount || burst.damage < kMultiHitMinDamage) return;
		burst.rolled = true;    // one roll per burst: the remaining pellets do not re-roll
		if (cooling()) return;  // also covers a heavy hit of this burst that already knocked
		const bool knock = Roll(kMultiHitChance);
		REX::INFO("DynamicRagdollSystem: {:08X} multi-hit: {} hits, {:.1f} damage in {:.3f}s; knockdown {}"sv, id, burst.hits, burst.damage,
			now - burst.start, knock ? "queued" : "not rolled");
		if (knock) QueueKnock(id, burst.dir, kMultiHitForce, now);
	}

	// Starts the queued knockdowns: ends a running hit reaction first (a driven reaction pins the body, as for the burn
	// fall), then the engine's explosion knockdown pushed along the hits.
	void ProcessPendingKnocks()
	{
		std::scoped_lock lock{ g_mutex };
		std::erase_if(g_pendingKnocks, [&](const PendingKnock& a_knock) {
			const auto actor = RE::TESForm::GetFormByID<RE::Actor>(a_knock.formID);
			// Dead (the death system has it) or already down: nothing to do.
			if (!actor || actor->IsDead(false) || actor->knockState != 0 || !actor->currentProcess || !actor->Get3D()) return true;
			if (const auto why = LockedReason(actor)) {
				LogSkipOnce(actor, "hit knockdown skipped", why);
				return true;
			}
			if (const auto it = std::find_if(g_stumbles.begin(), g_stumbles.end(), [&](const Stumble& a_s) { return a_s.formID == a_knock.formID; });
				it != g_stumbles.end()) {
				if (!EndStumble(*it, actor)) return false;  // world lock busy: retry next update
				SetAttackLock(*it, actor, false);
				g_stumbles.erase(it);
			}
			auto from = actor->GetPosition();
			from.x -= a_knock.dir.x * 50.0f;
			from.y -= a_knock.dir.y * 50.0f;
			from.z += 50.0f;
			actor->currentProcess->KnockExplosion(actor, from, a_knock.force);
			REX::INFO("DynamicRagdollSystem: {:08X} hit knockdown started (force {:.1f})"sv, a_knock.formID, a_knock.force);
			return true;
		});
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
		for (const auto& handle : lists->highActorHandles) {
			const auto actor = handle.get();
			if (!actor || actor->IsPlayerRef() || actor->IsDead(false) || actor->knockState == 0 || actor->knockState == kKnockGetUp ||
				actor->knockState == kKnockDown || !actor->Get3D() || IsBlacklisted(actor.get(), kKnockBlacklist)) continue;
			const auto id = actor->GetFormID();
			std::scoped_lock lock{ g_mutex };
			if (std::any_of(g_knockdowns.begin(), g_knockdowns.end(), [&](const Death& a_d) { return a_d.formID == id; }) ||
				std::any_of(g_deaths.begin(), g_deaths.end(), [&](const Death& a_d) { return a_d.formID == id; })) continue;
			Death down{};
			down.formID = id;
			down.start = a_now;
			down.stumble = false;
			down.grip = false;
			down.flail = Random(kFlailStrength.min, kFlailStrength.max);
			g_knockdowns.push_back(std::move(down));
			REX::INFO("DynamicRagdollSystem: {:08X} knockdown (knock state {}): brace / flail"sv, id, static_cast<std::uint32_t>(actor->knockState));
		}
	}


	// Living NPC in an engine ragdoll (explosion knockdown, push, fall): flying, the limbs flail; otherwise the free
	// hands brace as it tips over. The death brace and flail, without any balance help.
	void StepKnockdownFall(Death& a_down, RE::Actor* a_actor, float a_now)
	{
		auto* world = GetWorld(a_actor);
		auto* npWorld = world ? world->m_worldNP.get() : nullptr;
		if (!npWorld || !g_linearImpulse) return;
		WorldWriteLock lock{ npWorld };
		if (!lock.held) return;
		if (a_down.joints.empty()) {
			if (a_down.nextScan > a_now) return;
			a_down.nextScan = a_now + kScanInterval;
			ScanBodies(a_down, a_actor, npWorld);
			if (a_down.joints.empty()) return;
		}
		UpdateDetached(a_down, npWorld);
		auto& worldBodies = npWorld->m_bodyManager.m_bodies;
		std::vector<DeathSample> samples;
		for (const auto& body : a_down.bodies) {
			if (!FindBody(worldBodies, body)) continue;
			const auto& b = worldBodies[body.index];
			if (a_down.Detached(body.id.m_value) || !b.m_flags.all(RE::hknpCollisionFlagsEnum::kIsDynamic)) continue;
			MotionState motion{};
			if (ReadMotionState(npWorld, b, motion)) samples.push_back({ &body, motion, BodyPosition(b) });
		}
		if (samples.empty()) return;
		if (a_down.targets.empty()) {  // UpdateGround only casts once a pose was captured
			for (const auto& s : samples)
				a_down.targets.push_back({ s.ref->id.m_value, RotationFrom(worldBodies[s.ref->index].m_transform.m_rotation), s.pos });
			return;
		}
		const float dt = ControlDt(a_down.lastStep, a_now);
		const auto [low, high] = TrunkEnds(samples);  // pelvis / SPINE1, chest
		if (!low || !high || low == high || a_down.groundFeet <= 0) return;

		// Clearance: the lowest of the feet and the trunk above the ground under it. A missed ray reads as kRayDown
		// below the body, so a long drop counts as flying.
		float clearance = a_down.groundValid ? low->pos.z - a_down.groundTrunk : 1.0e9f;
		const float ground = a_down.groundValid ? a_down.groundTrunk : a_down.groundFoot[0];
		for (int k = 0; k < a_down.groundFeet; ++k)
			for (const auto& s : samples)
				if (s.ref->id.m_value == a_down.groundFootId[k]) clearance = std::min(clearance, s.pos.z - a_down.groundFoot[k]);
		if (clearance > 1.0e8f) return;
		const float flailAmount = std::clamp((clearance - kAirborneClear) / 0.2f, 0.0f, 1.0f);

		if (flailAmount > 0.0f && a_down.flail > 0.0f) {
			const auto flailing = FlailLimbs(npWorld, a_down, a_now - a_down.start, dt, flailAmount, a_down.flail * flailAmount, 0, 0);
			if (!a_down.flailLogged && flailing > 0) {
				a_down.flailLogged = true;
				REX::INFO("DynamicRagdollSystem: {:08X} knockdown: flying, flailing ({} limb joints, clearance {:.2f})"sv, a_down.formID, flailing, clearance);
			}
		}
		// Brace while tipping over above the ground; flying wins the arms.
		if (kDeathBrace > 0.0f && flailAmount < 1.0f && low->pos.z - ground > kBraceGround + 0.05f) {
			const float lean = std::acos(std::clamp(Unit(Sub(high->pos, low->pos)).z, -1.0f, 1.0f));
			ApplyBrace(a_down, npWorld, samples, dt, lean, 1.0f - flailAmount, ground);
		}
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
					if (!(body.pelvis || body.spine) || !FindBody(worldBodies, body) || a_d.Detached(body.id.m_value)) continue;
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
				!IsHumanoid(actor.get()) || InLockedAnimation(actor.get()) || IsBlacklisted(actor.get(), kKnockBlacklist) ||
				(!kKnockDownPowerArmor && RE::PowerArmor::ActorInPowerArmor(*actor))) continue;
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
