#include "Common.h"

namespace DynamicRagdollSystem
{
	// On fire: an active effect whose hit shader is one of Fallout4.esm's fire shaders, or with the DamageTypeFire
	// keyword (flamer, Shishkebab, incendiary, Molotov, fire hazards). Never for actors in power armor or effects from
	// abilities. shortcut: base-game forms only; mods that add their own fire shaders/keywords are not detected.
	bool IsBurning(RE::Actor* a_actor, std::string* a_why)
	{
		static constexpr std::array<RE::TESFormID, 12> kFireShaders{ 0x0014237E, 0x001B3F73, 0x0014237D, 0x00145365, 0x00149E5E,
			0x002407FB, 0x00231F8A, 0x00240C20, 0x00211112, 0x00216352, 0x0001D22B, 0x000303B0 };
		constexpr RE::TESFormID kDamageTypeFire = 0x0004A0C8;
		if (!kBurnPowerArmor && RE::PowerArmor::ActorInPowerArmor(*a_actor)) return false;  // bBurnPowerArmor off: never burning
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

	// Every 0.25 s: living humanoid NPCs on fire get (or keep) a burning reaction.
	void ScanForBurning(float a_now)
	{
		static float nextScan = 0.0f;
		if (!kBurnReaction || a_now < nextScan) return;
		nextScan = a_now + 0.25f;
		auto* lists = RE::ProcessLists::GetSingleton();
		if (!lists) return;
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
			const float a = Random(0.0f, 6.2831853f);
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
		if (!a_stumble.burnLogged) {
			a_stumble.burnLogged = true;
			REX::INFO("DynamicRagdollSystem: {:08X} burning: flailing and staggering"sv, a_stumble.formID);
		}

		// Shove: a random horizontal kick on the bodies above the centre of mass.
		if (a_now >= a_stumble.nextShove && kBurnShove > 0.0f) {
			a_stumble.nextShove = a_now + Random(kBurnShoveMin, kBurnShoveMax) * (1.0f - 0.5f * a_stumble.burnProgress);
			struct Sample { const BodyRef* ref; float inverseMass; Vec3 pos; };
			std::vector<Sample> samples;
			Vec3 com{};
			float mass = 0.0f;
			for (const auto& body : a_stumble.bodies) {
				if (!FindBody(worldBodies, body)) continue;
				MotionState motion{};
				if (!ReadMotionState(npWorld, worldBodies[body.index], motion)) continue;
				samples.push_back({ &body, motion.inverseMass, BodyPosition(worldBodies[body.index]) });
				com = Add(com, Scale(samples.back().pos, 1.0f / motion.inverseMass));
				mass += 1.0f / motion.inverseMass;
			}
			if (mass > 0.0f) {
				com = Scale(com, 1.0f / mass);
				const float a = Random() * 6.2831853f;
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

	// Per frame for a burning reaction: progress towards the fall, attack lock, and the knockdown when time is up.
	void UpdateBurn(Stumble& a_stumble, RE::Actor* a_actor, float a_now)
	{
		if (a_stumble.burnStart <= 0.0f) {
			a_stumble.burnStart = a_now;
			a_stumble.burnFallTime = Random(kBurnFallMin, kBurnFallMax);
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

	// After the burning reaction ended: the engine knockdown, and a burn-down entry that makes the body writhe
	// and roll on the ground (dying-phase twitches/rolls) while it is still burning.
	void StartBurnDown(RE::Actor* a_actor, float a_now)
	{
		// The engine's explosion knockdown (AIProcess::KnockExplosion → task 10): a real ragdoll pushed away from the
		// given point, then the usual get-up. kActionKnockdown only played a prone animation on living NPCs.
		const float angle = Random(0.0f, 6.2831853f);
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
		down.end = a_now + 5.0f;  // StepDeathStumble only scans the bodies (no stumble), then the dying phase runs
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
}
