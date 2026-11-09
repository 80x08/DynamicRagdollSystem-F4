#include "Common.h"

namespace DynamicRagdollSystem
{
	// [Knockdown] Living NPCs knocked over by bullet hits: a burst (shotgun pellets land within a few ms, each its own
	// hit) or one heavy hit. Explosion hits never count: explosions knock NPCs down by themselves. Hits are counted in
	// the hit hook after the engine applied them; the knockdown starts next update.

	static bool Roll(float a_chance)
	{
		static std::mt19937 rng{ std::random_device{}() };
		return std::uniform_real_distribution<float>{ 0.0f, 1.0f }(rng) < a_chance;
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
		if (IsTurret(a_actor) || (kHitKnockHumanoidOnly && !IsHumanoid(a_actor)) ||
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
}
