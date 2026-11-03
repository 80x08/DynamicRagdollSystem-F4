#include "Common.h"

namespace DynamicRagdollSystem
{

	void OnWorldUpdate(RE::bhkWorld* a_world)
	{
		const float now = NowSeconds();
		PollSettings(now);
		ScanForBurning(now);
		ProcessPending();
		{
			std::scoped_lock lock{ g_mutex };
			for (auto it = g_stumbles.begin(); it != g_stumbles.end();) {
				const auto actor = RE::TESForm::GetFormByID<RE::Actor>(it->formID);
				if (it->burning) {
					// Fire still on: keep the reaction going. Fire out: blend back to animation.
					const bool stillBurning = now <= it->burnUntil;
					if (stillBurning) it->end = std::max(it->end, now + 0.5f);
					else it->end = std::min(it->end, now + kBlendOut);
					it->burning = stillBurning;
					if (stillBurning && actor) {
						UpdateBurn(*it, actor, now);
					} else {
						SetAttackLock(*it, actor, false);
						it->burnStart = 0.0f;
						it->burnProgress = 0.0f;
					}
					std::uint64_t arms = 0;
					for (const auto& body : it->bodies)
						if (body.arm && body.bone >= 0 && body.bone < 64) arms |= std::uint64_t{ 1 } << body.bone;
					std::scoped_lock driveLock{ g_driveMutex };
					for (auto& [driver, state] : g_drives) {
						if (state.formID != it->formID) continue;
						state.end = it->end;
						state.burning = stillBurning;
						state.burnProgress = it->burnProgress;
						state.armBones = arms;
					}
				}
				if (it->burnFell && actor && !actor->IsDead(false) && actor->knockState == 0) {
					if (!EndStumble(*it, actor)) {  // world lock busy: retry next frame
						++it;
						continue;
					}
					SetAttackLock(*it, actor, false);
					StartBurnDown(actor, now);
					it = g_stumbles.erase(it);
					continue;
				}
				if (!actor || now >= it->end || actor->IsDead(false) || actor->knockState != 0 || IsMismatched(it->formID)) {
					if (!EndStumble(*it, actor)) {  // world lock busy: retry next frame
						++it;
						continue;
					}
					SetAttackLock(*it, actor, false);
					it = g_stumbles.erase(it);
					continue;
				}
				if (!it->pushed && GetWorld(actor) == a_world) {
					ApplyHitPush(*it, actor, now);
				}
				if (GetWorld(actor) == a_world) {
					EnforceNoCCLayer(*it, actor);
				}
				if (it->burning && it->pushed && GetWorld(actor) == a_world) {
					ApplyBurn(*it, actor, now);
				}
				if (GetWorld(actor) == a_world && now >= it->nextTrace) {
					TraceMovement(*it, actor, now);
				}
				++it;
			}
		}
		StartPendingDeaths(now);
		{
			std::scoped_lock lock{ g_mutex };
			for (auto it = g_deaths.begin(); it != g_deaths.end();) {
				const auto actor = RE::TESForm::GetFormByID<RE::Actor>(it->formID);
				const float dyingStart = it->downTime > 0.0f ? it->downTime : it->end + kDownTimeout;
				const bool revived = actor && !actor->IsDead(false) && !it->burnDown;  // resurrected (console/script) mid-death
				bool burnDownOver = false;
				if (it->burnDown && actor) {
					auto* middleHigh = actor->currentProcess ? actor->currentProcess->middleHigh : nullptr;
					if (actor->knockState != 0 && !it->seenDown) {
						it->seenDown = true;
						REX::INFO("DynamicRagdollSystem: {:08X} burning: down (knock state {}) after {:.2f}s"sv, it->formID,
							static_cast<std::uint32_t>(actor->knockState), now - it->start);
					}
					if (const auto knock = static_cast<std::uint32_t>(actor->knockState); knock != it->lastKnock) {
						REX::INFO("DynamicRagdollSystem: {:08X} burning: knock state {} -> {} at {:.2f}s (timer {:.2f}, burning {})"sv, it->formID,
							it->lastKnock, knock, now - it->start, middleHigh ? middleHigh->bleedoutTimer : -1.0f, IsBurning(actor));
						it->lastKnock = knock;
					}
					// The fire effect can drop for a moment (one effect ends as the next lands): only call it out
					// after kBurnOutGrace without fire.
					if (IsBurning(actor)) it->fireOutSince = 0.0f;
					else if (it->fireOutSince <= 0.0f) it->fireOutSince = now;
					const bool fireOut = it->fireOutSince > 0.0f && now - it->fireOutSince > kBurnOutGrace;
					// Stay down while burning: a paralyzed actor never gets up (knock update 0x140E73F80 skips the
					// get-up while Actor+0x2D0 bit 31 = Paralysis > 0). Once the fire is out the normal get-up follows.
					SetBurnParalysis(*it, actor, it->seenDown && !fireOut && !actor->IsDead(false));
					if (!it->seenDown && !it->fallFallback && now - it->start > 0.5f) {
						// kActionKnockdown did not take: the engine's instant ragdoll instead.
						it->fallFallback = true;
						const bool done = PerformDefaultAction(actor, RE::DEFAULT_OBJECT::kActionRagdollInstant);
						REX::INFO("DynamicRagdollSystem: {:08X} burning: no knockdown, instant ragdoll {}"sv, it->formID, done ? "started" : "failed");
					}
					burnDownOver = actor->IsDead(false) || (it->seenDown && actor->knockState == 0) || fireOut ||
					               (!it->seenDown && now - it->start > 1.5f) || now - it->start > kBurnDownMax;
				}
				const bool finishedOff = (kDyingEndDamage > 0.0f && it->dyingDamage >= kDyingEndDamage) || it->headCut;
				if (!actor || !actor->Get3D() || revived || burnDownOver || finishedOff || now >= dyingStart + it->dyingTime) {
					SetBurnParalysis(*it, actor, false);
					if (finishedOff) REX::INFO("DynamicRagdollSystem: {:08X} dying phase ended: {}"sv, it->formID,
						it->headCut ? "head dismembered"s : std::format("{:.0f} damage taken", it->dyingDamage));
					if (it->dyingLogged) {
						REX::INFO("DynamicRagdollSystem: {:08X} dying phase over: {} twitches, {} accepted joint impulses, {} rolls"sv,
							it->formID, it->twitchCount, it->twitchHits, it->rollCount);
					}
					it = g_deaths.erase(it);
					continue;
				}
				if (GetWorld(actor) == a_world) {
					// Burn-down: StepDeath only scans the bodies; go to the dying phase (writhing) right after.
					if (now < it->end && !(it->burnDown && !it->joints.empty())) {
						UpdateGround(*it, actor, a_world);
						StepDeath(*it, actor, now);
					} else {
						if (it->downTime <= 0.0f) UpdateGround(*it, actor, a_world);  // the brace needs the ground while it falls
						StepDying(*it, actor, now);
					}
				}
				++it;
			}
		}
		ScanForKnockdowns(now);
		{
			std::scoped_lock lock{ g_mutex };
			for (auto it = g_knockdowns.begin(); it != g_knockdowns.end();) {
				const auto actor = RE::TESForm::GetFormByID<RE::Actor>(it->formID);
				if (!actor || actor->IsDead(false) || actor->knockState == 0 || actor->knockState == kKnockGetUp || !actor->Get3D()) {
					it = g_knockdowns.erase(it);
					continue;
				}
				if (GetWorld(actor) == a_world) {
					UpdateGround(*it, actor, a_world);
					StepKnockdown(*it, actor, now);
				}
				++it;
			}
		}
		UpdateCollisionFilter();
		KnockOverStanding(a_world, now);
	}


	// Loading a save or starting a new game: drop all tracking (saved corpses and NPCs come back with new bodies).
	void ClearTracking()
	{
		ReleaseAttackLocks();
		{
			std::scoped_lock lock{ g_mutex };
			g_stumbles.clear();
			g_pending.clear();
			g_deaths.clear();
			g_knockdowns.clear();
			g_pendingDeaths.clear();
			g_lastHit.clear();
			g_lastTrigger.clear();
			g_deathMoveVel.clear();
			g_burningDeaths.clear();
		}
		std::scoped_lock lock{ g_driveMutex };
		g_drives.clear();
		g_mismatched.clear();  // form IDs of created refs (FF...) are reused across saves
	}


	bool Hook_WorldUpdate(RE::bhkWorld* a_world, std::uint32_t a_flags)
	{
		OnWorldUpdate(a_world);
		return (*g_worldUpdateHook)(a_world, a_flags);
	}


	bool Hook_WorldMUpdate(RE::bhkWorld* a_world, std::uint32_t a_flags)
	{
		static bool logged = false;
		if (!logged) {
			logged = true;
			REX::INFO("DynamicRagdollSystem: exterior physics world (bhkWorldM) update hooked"sv);
		}
		OnWorldUpdate(a_world);
		return (*g_worldMUpdateHook)(a_world, a_flags);
	}


	void InstallHooks()
	{
		ResolveImpulseFunction();
		InstallDriveToPoseHook();
		InstallPostPhysicsHook();
		static REL::THookVFT<bool(RE::bhkWorld*, std::uint32_t)> worldUpdateHook{
			REL::Offset{ RE::VTABLE::bhkWorld[0].offset() }, 0x28, &Hook_WorldUpdate
		};
		worldUpdateHook.Init();
		worldUpdateHook.Enable();
		g_worldUpdateHook = &worldUpdateHook;
		static REL::THookVFT<bool(RE::bhkWorld*, std::uint32_t)> worldMUpdateHook{
			REL::Offset{ RE::VTABLE::bhkWorldM[0].offset() }, 0x28, &Hook_WorldMUpdate
		};
		worldMUpdateHook.Init();
		worldMUpdateHook.Enable();
		g_worldMUpdateHook = &worldMUpdateHook;
		InstallMovementHook();
		InstallGraphUpdateHook();
		InstallHitHook();
		static REL::THookVFT<void(RE::Actor*, RE::Actor*, float, bool, bool)> killHook{
			REL::Offset{ RE::VTABLE::Actor[0].offset() }, 0x117, &Hook_KillImpl
		};
		killHook.Init();
		killHook.Enable();
		g_killHook = &killHook;
		REX::INFO("DynamicRagdollSystem: hooks installed"sv);
	}
}
