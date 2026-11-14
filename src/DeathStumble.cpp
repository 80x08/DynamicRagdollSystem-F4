#include "Common.h"

// Death stumble ([DeathStumble]): the standing phase of a death. StepDeathStumble runs every Havok step while the dead
// body is on its feet: it drives the death walk (Walk.cpp) or the death stagger (DeathStagger.cpp), or else its own
// balance controller, plus the killing push, grip, brace and flail. The dying phase follows (StepWrithe, Death.cpp).

namespace DynamicRagdollSystem
{
	// Weapon knockback: guns with a big impact force launch the corpse faster than it can be caught. For a short window,
	// speed beyond the pre-death movement (sideways, or upward) is cut to fDeathKnockbackScale of the largest seen.
	// Tracking the peak keeps it from compounding frame to frame. Not for explosions. Updates comVel.
	void TrimKnockback(Death& a_stumble, RE::hknpBSWorld* npWorld, const std::vector<DeathSample>& samples, Vec3& comVel, float a_now)
	{
		if (a_now - a_stumble.start >= kKnockWindow || a_stumble.flung || kDeathKnockbackScale >= 1.0f) return;
		Vec3 extra = Sub(comVel, a_stumble.moveVel);
		extra.z = std::max(extra.z, 0.0f);  // falling is not knockback
		const float size = Length(extra);
		a_stumble.knockPeak = std::max(a_stumble.knockPeak, size);
		const float allowed = kDeathKnockbackScale * a_stumble.knockPeak;
		if (size > allowed + 0.05f) {
			const auto dv = Scale(extra, allowed / size - 1.0f);
			for (const auto& s : samples) {
				const auto impulse = Scale(dv, 1.0f / s.motion.inverseMass);
				RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
				g_linearImpulse(npWorld, s.ref->id.m_value, &nativeImpulse);
			}
			comVel = Add(comVel, dv);
			if (!a_stumble.knockLogged) {
				a_stumble.knockLogged = true;
				REX::INFO("DynamicRagdollSystem: {:08X} knockback trimmed: extra speed {:.2f} -> {:.2f} (moving {:.2f} before death, weapon '{}', {:.0f} damage)"sv,
					a_stumble.formID, size, allowed, Length(a_stumble.moveVel), a_stumble.weapon, a_stumble.damage);
			}
		}
	}

	// One-off push from the killing hit, spread like the reaction push (PushSpeed). No push without a recent hit.
	void ApplyKillingPush(Death& a_stumble, RE::hknpBSWorld* npWorld, const std::vector<DeathSample>& samples, Vec3 com, float mass, float a_scale)
	{
		if (a_stumble.hitApplied || samples.empty() || mass <= 0.0f) return;
		a_stumble.hitApplied = true;
		const bool hitKnown = std::any_of(samples.begin(), samples.end(),
			[&](const DeathSample& a_s) { return a_s.ref->id.m_value == a_stumble.hitBody; });
		float lowest = std::numeric_limits<float>::max();
		for (const auto& s : samples) lowest = std::min(lowest, s.pos.z);
		Vec3 ownPush{};
		for (const auto& s : samples) {
			const float push = PushSpeed(a_stumble, *s.ref, hitKnown, s.pos.z, com.z) * a_scale;
			if (push <= 0.0f) continue;
			auto impulse = Scale(a_stumble.hitDir, push / s.motion.inverseMass);
			if (s.pos.z - lowest < kFloorBodyHeight) impulse.z = std::max(impulse.z, 0.0f);  // floor-level bodies: not into the ground
			ownPush = Add(ownPush, impulse);
			RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
			g_linearImpulse(npWorld, s.ref->id.m_value, &nativeImpulse);
		}
		// Our own push is not weapon knockback: add it to the baseline so the knockback trim leaves it alone.
		// Not its downward part: the trim ignores falling, and a lowered baseline would read as upward knockback.
		ownPush.z = std::max(ownPush.z, 0.0f);
		a_stumble.moveVel = Add(a_stumble.moveVel, Scale(ownPush, 1.0f / mass));
		// Blast: the same velocity on every body part, away from the blast and upward.
		if (a_stumble.explosionPush > 0.0f) {
			const Vec3 blast{ a_stumble.hitDir.x * a_stumble.explosionPush, a_stumble.hitDir.y * a_stumble.explosionPush,
				kExplosionLift * a_stumble.explosionPush };
			for (const auto& s : samples) {
				const auto impulse = Scale(blast, 1.0f / s.motion.inverseMass);
				RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
				g_linearImpulse(npWorld, s.ref->id.m_value, &nativeImpulse);
			}
		}
	}

	// How hard the limbs flail (0..1), for the stagger and the balance controller: while the body cannot be caught (faster
	// than a_catchSpeed, fully at + a_catchRange) or is off the ground, until it lands; also while falling off a ledge.
	// a_feet: the feet's midpoint, nullptr when there is none. Heights are above the ground it stood on.
	static float FlailAmount(Death& a_stumble, Vec3 a_com, Vec3 a_vel, const Vec3* a_feet, float a_feetLevel, float a_trunkLevel,
		float a_catchSpeed, float a_catchRange)
	{
		// Off the ground = feet up *and* the COM at least near standing height: buckling knees fold the feet up in a collapse.
		if (a_stumble.standHeight <= 0.0f) a_stumble.standHeight = a_com.z - a_trunkLevel;
		const bool airborne = a_feet && a_feet->z - a_feetLevel >= kGroundTolerance && a_com.z - a_trunkLevel > 0.9f * a_stumble.standHeight;
		// Height above the ground it stood on (not above its own feet: a tumbling body has its feet level with it).
		const bool  landed = a_com.z - a_trunkLevel < kFlailGround;
		const float speed = std::sqrt(a_vel.x * a_vel.x + a_vel.y * a_vel.y);
		const float fast = std::clamp((speed - a_catchSpeed) / std::max(a_catchRange, 0.1f), 0.0f, 1.0f);
		// Falling (e.g. off a ledge, feet below the captured ground) also flails, until the drop slows down.
		// Only below the ground it stood on, so an ordinary collapse to the floor does not flail on the way down.
		const float falling = a_com.z - a_trunkLevel < -0.3f ? std::clamp((-a_vel.z - 1.5f) / 1.5f, 0.0f, 1.0f) : 0.0f;
		return std::max(landed ? 0.0f : std::max(fast, airborne ? 1.0f : 0.0f), falling);
	}

	void StepDeathStumble(Death& a_stumble, RE::Actor* a_actor, float a_now)
	{
		auto* world = GetWorld(a_actor);
		auto* npWorld = world ? world->m_worldNP.get() : nullptr;
		if (!npWorld || !g_linearImpulse) return;

		WorldWriteLock lock{ npWorld };
		if (!lock.held) {
			if (!a_stumble.writerGateLogged) {
				a_stumble.writerGateLogged = true;
				REX::INFO("DynamicRagdollSystem: world lock busy; skipping frame"sv);
			}
			return;
		}

		auto& worldBodies = npWorld->m_bodyManager.m_bodies;
		const auto valid = [&](const BodyRef& a_ref) {
			return FindBody(worldBodies, a_ref) && !a_stumble.Detached(a_ref.id.m_value);
		};
		const auto feetMidpoint = [&](Vec3& a_out) {
			Vec3 sum{};
			int  count = 0;
			for (const auto& body : a_stumble.bodies) {
				if (body.foot && valid(body)) {
					sum = Add(sum, BodyPosition(worldBodies[body.index]));
					++count;
				}
			}
			if (count) a_out = Scale(sum, 1.0f / count);
			return count > 0;
		};

		// Constraints appear a frame or two after the knock; keep scanning until the joints are there.
		if (a_stumble.joints.empty()) {
			if (a_stumble.nextScan > a_now) return;
			a_stumble.nextScan = a_now + kScanInterval;
			ScanBodies(a_stumble, a_actor, npWorld);
		}
		// Capture on the first frame bodies exist: they still hold the animated standing pose then.
		if (a_stumble.targets.empty() && !a_stumble.bodies.empty()) {
			for (const auto& body : a_stumble.bodies) {
				if (!valid(body)) continue;
				const auto& b = worldBodies[body.index];
				//store rotation of bodies in death pose as reference
				a_stumble.targets.push_back({ body.id.m_value, RotationFrom(b.m_transform.m_rotation), BodyPosition(b) });
			}
			a_stumble.hasFeet = feetMidpoint(a_stumble.feetCaptured); //averages the positions of the valid foot bodies
			// Death stagger: height of the lower foot (dying mid-stride one is in the air: the average put the ground ~5 cm
			// up, so that foot never counted as lifted); and the foot mid-swing at death (clearly faster than the other),
			// which takes the first step.
			float footSpeed[2]{};
			std::uint32_t footId[2]{};
			int footCount = 0;
			for (const auto& body : a_stumble.bodies) {
				if (!a_stumble.stagger || !body.foot || !valid(body) || footCount >= 2) continue;
				const auto& b = worldBodies[body.index];
				a_stumble.feetCaptured.z = std::min(a_stumble.feetCaptured.z, BodyPosition(b).z);
				MotionState motion{};
				if (ReadMotionState(npWorld, b, motion)) footSpeed[footCount] = std::sqrt(motion.linear.x * motion.linear.x + motion.linear.y * motion.linear.y);
				footId[footCount++] = body.id.m_value;
			}
			if (footCount == 2 && std::abs(footSpeed[0] - footSpeed[1]) > kSwingingFoot)
				a_stumble.swingAtDeath = footId[footSpeed[0] > footSpeed[1] ? 0 : 1];
			REX::INFO("DynamicRagdollSystem: {:08X} death pose captured after {:.2f}s ({} bodies, feet {}, gravity z {:.2f})"sv, a_stumble.formID,
				a_now - a_stumble.start, a_stumble.targets.size(), a_stumble.hasFeet, npWorld->m_gravity.z);
		}
		if (a_stumble.targets.empty()) return;
		UpdateDetached(a_stumble, npWorld); //check for any dismembered limb
		// Walk -> stumble: the stumble picks up at the walk's remaining strength (its fade is (end - now) / (end - stumbleStart)).
		const auto walkToStumble = [&] {
			const float strength = DeathWalkDrain(a_stumble, a_now);
			const float time = Random(kDeathStumbleTime.min, kDeathStumbleTime.max);
			EndDeathWalk(a_stumble.formID);
			a_stumble.walk = false;
			a_stumble.stumble = strength > 0.05f;  // a walk already drained out just collapses
			a_stumble.end = a_stumble.stumble ? a_now + time : a_now;
			a_stumble.stumbleStart = a_now - time * (1.0f / std::max(strength, 0.05f) - 1.0f);
		};
		if (a_stumble.walk && kDeathWalkEndDamage > 0.0f && a_stumble.walkDamage >= kDeathWalkEndDamage) {
			// Shot enough while walking: it stumbles on the hit, then collapses into the dying phase.
			REX::INFO("DynamicRagdollSystem: {:08X} death walk: {:.0f} damage taken; stumbles (strength {:.2f})"sv, a_stumble.formID,
				a_stumble.walkDamage, DeathWalkDrain(a_stumble, a_now));
			walkToStumble();
			return;
		}
		if (a_stumble.walk) {  // death walk instead of the stumble (Walk.cpp); the hand keeps clutching the wound
			const float dt = ControlDt(a_stumble.lastStep, a_now);
			std::vector<DeathSample> samples;
			Vec3 com{}, comVel{};
			float mass = 0.0f;
			for (const auto& body : a_stumble.bodies) {
				if (!valid(body)) continue;
				MotionState motion{};
				if (!ReadMotionState(npWorld, worldBodies[body.index], motion)) continue;
				samples.push_back({ &body, motion, BodyPosition(worldBodies[body.index]) });
				com = Add(com, Scale(samples.back().pos, 1.0f / motion.inverseMass));
				comVel = Add(comVel, Scale(motion.linear, 1.0f / motion.inverseMass));
				mass += 1.0f / motion.inverseMass;
			}
			// The gun's own knockback, cut like the stumble's (head shots launch it otherwise).
			if (mass > 0.0f) {
				comVel = Scale(comVel, 1.0f / mass);
				TrimKnockback(a_stumble, npWorld, samples, comVel, a_now);
			}
			// The killing hit's push, while the walk's support is knocked down (Walk.cpp): it visibly staggers first.
			if (mass > 0.0f) ApplyKillingPush(a_stumble, npWorld, samples, Scale(com, 1.0f / mass), mass, kDeathWalkPushScale);
			const auto result = StepDeathWalk(a_stumble, a_actor, npWorld, dt, a_now);
			if (result == WalkResult::kWalking && a_stumble.grip) {
				ApplyGrip(a_stumble, npWorld, samples, dt, 1.0f);
			} else if (result == WalkResult::kFallen) {
				a_stumble.walk = false;
				a_stumble.end = a_now;  // collapse: the dying phase follows once it lies still
			} else if (result == WalkResult::kUnable) {
				walkToStumble();  // the stumble it would have had instead
			}
			return;
		}
		if (!a_stumble.stumble) { //end if didnt roll stumble
			a_stumble.end = a_now;  // plain ragdoll fall
			return;
		}

		const auto findTarget = [&](std::uint32_t a_id) -> const BodyTarget* {
			for (const auto& target : a_stumble.targets)
				if (target.id == a_id) return &target;
			return nullptr;
		};

		const float dt = ControlDt(a_stumble.lastStep, a_now);
		// Strength decays steadily from the moment of death, so the body buckles progressively instead of
		// standing at full strength and dropping at the end.
		const float fadeFrom = a_stumble.stumbleStart != 0.0f ? a_stumble.stumbleStart : a_stumble.start;
		const float fade = std::clamp((a_stumble.end - a_now) / std::max(a_stumble.end - fadeFrom, 0.01f), 0.0f, 1.0f);
		if (fade <= 0.0f) return;

		// 1. Whole-rig balance. Every body gets the velocity change of one rigid body, so the
		//    whole mass is moved (no per-body pinning, which froze the rig into a statue):
		//    - tilt: rotate about the feet toward the captured trunk tilt (yaw left free),
		//    - height: spring the centre of mass to its captured height above the feet, cancel gravity,
		//    - horizontal: only light damping, so the hit's momentum turns into a stumble.
		// dCom - desired linear velocity
		// dOmega - desired angluar velocity
		using Sample = DeathSample;
		std::vector<Sample> samples;
		std::int32_t notDynamic = 0, noMotion = 0;
		const BodyRef* trunk = nullptr;
		Vec3 com{}, comVel{};
		float mass = 0.0f;
		for (const auto& body : a_stumble.bodies) {
			if (!valid(body)) continue;
			const auto& b = worldBodies[body.index];
			if (!b.m_flags.all(RE::hknpCollisionFlagsEnum::kIsDynamic)) { ++notDynamic; continue; }
			MotionState motion{};
			if (!ReadMotionState(npWorld, b, motion)) { ++noMotion; continue; }
			const float m = 1.0f / motion.inverseMass;
			samples.push_back({ &body, motion, BodyPosition(b) });
			com = Add(com, Scale(samples.back().pos, m));
			comVel = Add(comVel, Scale(motion.linear, m));
			mass += m;
			if (!trunk || body.priority < trunk->priority) trunk = &body;
		}
		Vec3 feet{};
		const BodyTarget* trunkTarget = trunk ? findTarget(trunk->id.m_value) : nullptr;
		std::int32_t pushed = 0;
		Vec3 tiltError{};
		bool grounded = false;
		float flailAmount = 0.0f;  // 0..1: how much the body is flying rather than standing
		float braceLean = 0.0f;    // lean for the brace (set while the body is still above the ground)
		// Ground levels (foot-centre height when standing) from the rays, as shifts from the ground at death.
		const bool rays = a_stumble.groundCaptured && a_stumble.groundValid;
		const float rest = a_stumble.feetCaptured.z;  // foot-centre height when standing at death
		const auto footLevel = [&](std::uint32_t a_id) {
			if (rays)
				for (int k = 0; k < 2; ++k)
					if (a_stumble.groundFootId[k] == a_id) return rest + a_stumble.groundFoot[k] - a_stumble.groundAtDeath;
			return rest;
		};
		const float feetGround = a_stumble.groundFeet == 2 ? 0.5f * (a_stumble.groundFoot[0] + a_stumble.groundFoot[1]) : a_stumble.groundFoot[0];
		const float feetLevel = rays ? rest + feetGround - a_stumble.groundAtDeath : rest;
		const float trunkLevel = rays ? rest + a_stumble.groundTrunk - a_stumble.groundAtDeath : rest;
		const float groundSurface = rays ? a_stumble.groundTrunk : rest - 0.1f;

		// Death stagger (Death::stagger): the walker staggering under the hit (Walk.cpp, StepDeathStagger) instead of the
		// balance controller below, which still runs for the death stumble, flung deaths (never staggering) and a skeleton
		// the walker can't stand on. Around it, as below: knockback trim, killing push, grip, brace, flail.
		if (a_stumble.stagger && !a_stumble.flung && mass > 0.0f && trunk && trunk->priority < 99) {
			const Vec3 centre = Scale(com, 1.0f / mass);
			Vec3 velocity = Scale(comVel, 1.0f / mass);
			TrimKnockback(a_stumble, npWorld, samples, velocity, a_now);  // before the wait: its window is as short
			comVel = Scale(velocity, mass);  // trimmed, for the balance controller below if the stagger can't start
			if (!a_stumble.groundCaptured && a_now - a_stumble.start < kGroundWait) return;  // the ground ray comes a frame later
			const auto* pelvis = &*std::find_if(samples.begin(), samples.end(), [&](const Sample& a_s) { return a_s.ref == trunk; });
			// Faster than fDeathStaggerCatchSpeed (+fDeathStaggerCatchRange), or leaning past fDeathStaggerGiveUpTilt, cannot be caught: the help fades into a fall.
			float lean = 0.0f;
			if (trunkTarget) {
				auto error = RotationError(RotationFrom(worldBodies[trunk->index].m_transform.m_rotation), trunkTarget->rot);
				error.z = 0.0f;
				lean = Length(error);
			}
			const float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
			const float help = std::clamp(1.0f - (speed - kDeathStaggerCatchSpeed) / std::max(kDeathStaggerCatchRange, 0.01f), 0.0f, 1.0f) *
			                   std::clamp((kDeathStaggerGiveUpTilt - lean) / kGiveUpRange, 0.0f, 1.0f);
			// Going down, the drives fade with the pelvis's height above the feet' ground, so it is limp once down, with
			// nothing cut (or when the time is up: fade).
			const float standing = trunkTarget ? trunkTarget->pos.z - rest : 0.0f;
			const float up = a_stumble.hitApplied && standing > 0.1f ?
				std::clamp(((pelvis->pos.z - trunkLevel) / standing - kStaggerDown) / (kStaggerFalling - kStaggerDown), 0.0f, 1.0f) : 1.0f;
			// The stagger line: the way the body moves just after the shot (its push plus whatever it was doing, e.g.
			// walking). It tips over that way.
			if (Length(a_stumble.staggerLine) < 0.5f && a_stumble.hitApplied) {
				a_stumble.staggerLine = Unit(Vec3{ velocity.x, velocity.y, 0.0f });
				if (Length(a_stumble.staggerLine) < 0.5f) a_stumble.staggerLine = Unit(Vec3{ a_stumble.hitDir.x, a_stumble.hitDir.y, 0.0f });
				if (Length(a_stumble.staggerLine) < 0.5f) a_stumble.staggerLine = Vec3{ 0.0f, 1.0f, 0.0f };
			}
			const auto result = up > 0.0f ? StepDeathStagger(a_stumble, a_actor, npWorld, dt, a_now, fade, help * up) : WalkResult::kFallen;
			if (result == WalkResult::kFallen) {
				REX::INFO("DynamicRagdollSystem: {:08X} death stagger over after {:.2f}s, {} steps (pelvis {:.2f} up, lean {:.2f}, speed {:.2f}); limp"sv,
					a_stumble.formID, a_now - a_stumble.start, a_stumble.steps, pelvis->pos.z - trunkLevel, lean, speed);
				EndDeathWalk(a_stumble.formID);
				a_stumble.end = a_now;  // down: the dying phase follows
				return;
			}
			if (result == WalkResult::kWalking) {
				ApplyKillingPush(a_stumble, npWorld, samples, centre, mass, kDeathPushScale);
				Vec3        feetAt{};
				const float flailing = FlailAmount(a_stumble, centre, velocity, feetMidpoint(feetAt) ? &feetAt : nullptr, feetLevel, trunkLevel,
					kDeathStaggerCatchSpeed, kDeathStaggerCatchRange);
				if (a_stumble.grip) ApplyGrip(a_stumble, npWorld, samples, dt, 1.0f);
				a_stumble.braceAmount = 0.0f;
				a_stumble.braceOff = -1.0f;
				if (kDeathBrace > 0.0f)
					ApplyBrace(a_stumble, npWorld, samples, dt, centre.z - trunkLevel > kBraceGround ? lean : 0.0f, 1.0f - std::min(flailing, 1.0f), groundSurface);
				if (flailing > 0.0f && a_stumble.flail > 0.0f) {
					char gripSide = 0;
					if (a_stumble.grip)
						for (const auto& body : a_stumble.bodies)
							if (body.id.m_value == a_stumble.gripHand) gripSide = body.side;
					const auto limbs = FlailLimbs(npWorld, a_stumble, a_now - a_stumble.start, dt, flailing, a_stumble.flail * flailing, gripSide,
						a_stumble.stepFoot ? a_stumble.stepSide : 0);
					if (!a_stumble.flailLogged && limbs > 0) {
						a_stumble.flailLogged = true;
						REX::INFO("DynamicRagdollSystem: {:08X} flailing ({} limb joints, amount {:.2f})"sv, a_stumble.formID, limbs, flailing);
					}
				}
				if (++a_stumble.balanceSteps == 1 || a_stumble.balanceSteps % 30 == 0)
					REX::INFO("DynamicRagdollSystem: {:08X} death stagger {}: fade {:.2f} help {:.2f} up {:.2f} lean {:.2f} speed {:.2f} pelvis {:.2f} up, {} steps, flail {:.2f} brace {:.2f}"sv,
						a_stumble.formID, a_stumble.balanceSteps, fade, help, up, lean, speed, pelvis->pos.z - trunkLevel, a_stumble.steps, flailing,
						a_stumble.braceAmount);
				return;
			}
			a_stumble.stagger = false;  // kUnable: the balance controller below from now on
		}
		float overFeetLog = -1.0f; // ledge check (COM over the planted feet), for the log; -1 = no planted foot
		if (mass > 0.0f && trunkTarget && trunk->priority < 99 && feetMidpoint(feet)) {
			com = Scale(com, 1.0f / mass);
			comVel = Scale(comVel, 1.0f / mass);
			TrimKnockback(a_stumble, npWorld, samples, comVel, a_now);
			const auto& trunkMotion = std::find_if(samples.begin(), samples.end(),
				[&](const Sample& a_s) { return a_s.ref == trunk; })->motion;
			tiltError = RotationError(RotationFrom(worldBodies[trunk->index].m_transform.m_rotation), trunkTarget->rot);
			tiltError.z = 0.0f;
			auto omega = Rotate(trunkMotion.orientation, trunkMotion.angular);
			const float yawRate = omega.z;
			omega.z = 0.0f;
			// A body moving faster than a stumble cannot catch itself: above kCatchSpeed the upright torque,
			// weight support, braking and stepping fade out, so momentum carries it into a natural fall
			// (holding the torso up while the body slid on looked like floating).
			const float speed = std::sqrt(comVel.x * comVel.x + comVel.y * comVel.y);
			// External help (upright torque, weight support) only while the body is still nearly upright. Holding up
			// a body already leaning 40–85° read as an invisible hand; past kGiveUpTilt only the joint stiffness
			// (internal, muscle-like) remains and the body collapses.
			const float lean = Length(tiltError);
			const float upright = std::clamp((kGiveUpTilt - lean) / kGiveUpRange, 0.0f, 1.0f);
			const float catchable = std::clamp(1.0f - (speed - kCatchSpeed) / kCatchRange, 0.0f, 1.0f) * upright;
			flailAmount = FlailAmount(a_stumble, com, comVel, &feet, feetLevel, trunkLevel, kCatchSpeed, kCatchRange);
			Vec3 dOmega = Scale(Add(Scale(tiltError, -kTiltKp * a_stumble.upright), Scale(omega, -kTiltKd * a_stumble.upright)), dt * fade * catchable);
			if (const float n = Length(dOmega); n > kTiltMaxDeltaOmega) dOmega = Scale(dOmega, kTiltMaxDeltaOmega / n);
			// Yaw has no target (facing may change) but is damped, otherwise hits set the rig spinning.
			dOmega.z = std::clamp(-kYawKd * yawRate * dt * fade * catchable, -kTiltMaxDeltaOmega, kTiltMaxDeltaOmega);

			// Absolute heights: the captured ground is where the feet stood when hit. Measuring relative
			// to the feet let a launched rig float, since the feet rise with it.
			const float capturedComZ = [&] {
				float z = 0.0f, m = 0.0f;
				for (const auto& s : samples)
					if (const auto* t = findTarget(s.ref->id.m_value)) {
						const float w = 1.0f / s.motion.inverseMass;
						z += t->pos.z * w;
						m += w;
					}
				return m > 0.0f ? z / m : com.z;
			}();
			// Any foot on its own ground (the two-foot average dropped support while one foot was still planted).
			// Two-sided: feet that sank through the ground are not standing (one-sided kept supporting a sinking body).
			for (const auto& s : samples)
				if (s.ref->foot && std::abs(s.pos.z - footLevel(s.ref->id.m_value)) < kGroundTolerance) grounded = true;
			if (com.z - trunkLevel > kBraceGround) braceLean = lean;
			// The body sinks with the knee buckle instead of being held at standing height (which fought the bend).
			const float sink = kSinkDepth * std::min(1.0f, 1.5f * (1.0f - fade)) * std::clamp(kDeathKneeBend / 80.0f, 0.0f, 1.5f);
			const auto& g = npWorld->m_gravity;
			// Support (gravity cancel + height spring) only while the feet are on the ground; airborne, gravity acts.
			const float support = grounded ?
				(kHeightKp * (capturedComZ + (feetLevel - rest) - sink - com.z) - kHeightKd * comVel.z - g.z * a_stumble.support) * dt * fade * catchable : 0.0f;
			const Vec3 dCom{ -a_stumble.braking * comVel.x * dt * catchable, -a_stumble.braking * comVel.y * dt * catchable,
				grounded ? support : std::min(0.0f, -kHeightKd * comVel.z * dt) };  // airborne: brake the launch only
			// Stepping: when the capture point (where the COM will come to rest) leaves the feet,
			// swing the farther foot under it.
			if (grounded && !a_stumble.stepFoot && a_now >= a_stumble.nextStep && fade > 0.3f && catchable > 0.3f) {
				const Sample* footSamples[2]{};
				int footCount = 0;
				for (const auto& s : samples)
					if (s.ref->foot && footCount < 2) footSamples[footCount++] = &s;
				if (footCount == 2) {
					const float look = std::sqrt(std::max(com.z - feet.z, 0.3f) / std::max(-g.z, 1.0f));
					const Vec3 capture{ com.x + comVel.x * look, com.y + comVel.y * look, 0.0f };
					// Distance from the capture point to the segment between the feet (the support area).
					// Measuring from the midpoint kept triggering after every step, since a landed step
					// only moves the midpoint halfway.
					const Vec3 a{ footSamples[0]->pos.x, footSamples[0]->pos.y, 0.0f };
					const Vec3 b{ footSamples[1]->pos.x, footSamples[1]->pos.y, 0.0f };
					const auto ab = Sub(b, a);
					const float abLen2 = Dot(ab, ab);
					const float t = abLen2 > 1.0e-6f ? std::clamp(Dot(Sub(capture, a), ab) / abLen2, 0.0f, 1.0f) : 0.5f;
					const auto offset = Sub(capture, Add(a, Scale(ab, t)));
					if (Length(offset) > kStepTrigger) {
						const auto flatDist = [&](const Sample* a_s) {
							return Length(Vec3{ a_s->pos.x - capture.x, a_s->pos.y - capture.y, 0.0f });
						};
						// Alternate feet like real stumbling; only the first step picks the foot farther from the
						// capture point (always picking the farther one stepped the same foot again and again).
						bool firstSwings = flatDist(footSamples[0]) > flatDist(footSamples[1]);
						if (a_stumble.lastStepSide && footSamples[0]->ref->side != footSamples[1]->ref->side) {
							firstSwings = footSamples[0]->ref->side != a_stumble.lastStepSide;
						}
						const Sample* swing = footSamples[firstSwings ? 0 : 1];
						const Sample* stance = footSamples[firstSwings ? 1 : 0];
						// Land under the capture point but keep half the stance width sideways.
						const auto dir = Unit(offset);
						Vec3 width = Sub(swing->pos, stance->pos);
						width.z = 0.0f;
						width = Sub(width, Scale(dir, Dot(width, dir)));
						Vec3 step = Sub(Add(capture, Scale(width, 0.5f)), swing->pos);
						step.z = 0.0f;
						// Faster bodies take longer strides so the feet keep up instead of the body sliding past them.
						const float maxStep = std::min(kStepMaxLength * (1.0f + 0.25f * speed), 2.0f * kStepMaxLength);
						if (const float n = Length(step); n > maxStep) step = Scale(step, maxStep / n);
						else if (n > 1.0e-3f && n < kStepMinLength) step = Scale(step, kStepMinLength / n);
						a_stumble.stepFoot = swing->ref->id.m_value;
						a_stumble.stepSide = swing->ref->side;
						a_stumble.stepFrom = swing->pos;
						a_stumble.stepTo = Add(swing->pos, step);
						a_stumble.stepStart = a_now;
						a_stumble.stepGroundTried = false;  // landing ray next frame (UpdateGround)
						a_stumble.stepGroundHit = false;
						a_stumble.lastStepSide = swing->ref->side;
						a_stumble.stepTime = kStepTime / (1.0f + 0.3f * speed);  // quicker steps when moving fast
						REX::INFO("DynamicRagdollSystem: {:08X} death step {} with {} foot, length {:.2f}"sv, a_stumble.formID,
							++a_stumble.steps, swing->ref->side ? swing->ref->side : '?', Length(step));
					}
				}
			}
			if (a_stumble.stepFoot) {
				const float phase = (a_now - a_stumble.stepStart) / a_stumble.stepTime;
				const auto swing = std::find_if(samples.begin(), samples.end(),
					[&](const Sample& a_s) { return a_s.ref->id.m_value == a_stumble.stepFoot; });
				// Drop the step once the body can no longer catch itself: a full-strength swing kept yanking the foot
				// toward its target while the rest of the rig went limp at the end of the stumble.
				if (swing == samples.end() || phase >= 1.0f || !grounded || catchable < 0.3f || fade < 0.15f) {
					a_stumble.stepFoot = 0;
					a_stumble.stepSide = 0;
					a_stumble.nextStep = a_now + kStepCooldown;
				} else {
					const auto& from = a_stumble.stepFrom;
					const auto& to = a_stumble.stepTo;
					// Land at standing height on the ground under the target (as footLevel), so stairs and slopes are
					// stepped onto. No hit, or a change too big to be a step: the start height, as before.
					float toZ = from.z;
					if (rays && a_stumble.stepGroundHit) {
						const float level = rest + a_stumble.stepGround - a_stumble.groundAtDeath;
						if (std::abs(level - from.z) < kStepMaxRise) toZ = level;
					}
					const Vec3 desiredPos{ from.x + (to.x - from.x) * phase, from.y + (to.y - from.y) * phase,
						from.z + (toZ - from.z) * phase + kStepHeight * std::sin(3.14159265f * phase) };
					const Vec3 feedForward{ (to.x - from.x) / a_stumble.stepTime, (to.y - from.y) / a_stumble.stepTime,
						(toZ - from.z) / a_stumble.stepTime };
					const auto desiredV = Add(feedForward, Scale(Sub(desiredPos, swing->pos), kSwingKp));
					Vec3 deltaV = Sub(desiredV, swing->motion.linear);
					const float n = Length(deltaV), cap = kSwingMaxAccel * dt;
					if (std::isfinite(n) && n > 1.0e-4f) {
						if (n > cap) deltaV = Scale(deltaV, cap / n);
						const auto impulse = Scale(deltaV, 1.0f / swing->motion.inverseMass);
						RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
						g_linearImpulse(npWorld, a_stumble.stepFoot, &nativeImpulse);
						// Hip-muscle reaction on the trunk: an unreacted swing push added momentum toward the capture
						// point every step, so the body sped up (longer steps, more push) and slid past its planted leg.
						float trunkMass = 0.0f;
						for (const auto& s : samples)
							if (s.ref->pelvis || s.ref->spine) trunkMass += 1.0f / s.motion.inverseMass;
						if (trunkMass > 0.0f) {
							const auto backV = Scale(impulse, -1.0f / trunkMass);  // same velocity change on each trunk body
							for (const auto& s : samples) {
								if (!s.ref->pelvis && !s.ref->spine) continue;
								const auto back = Scale(backV, 1.0f / s.motion.inverseMass);
								RE::hkVector4f nativeBack{ back.x, back.y, back.z, 0.0f };
								g_linearImpulse(npWorld, s.ref->id.m_value, &nativeBack);
							}
						}
					}
				}
			}

			// All help acts through the planted feet: the opposite impulse goes into the stance feet, where the ground
			// takes it (like a leg pushing). No planted foot = no help, and sideways help is limited by foot friction,
			// so the body cannot hover or be held up against its momentum by an invisible hand.
			std::vector<const Sample*> stance;
			for (const auto& s : samples)
				if (s.ref->foot && s.ref->id.m_value != a_stumble.stepFoot && std::abs(s.pos.z - footLevel(s.ref->id.m_value)) < kGroundTolerance)
					stance.push_back(&s);
			// A body cannot hold itself up with its weight hanging past its feet (e.g. over a ledge): fade all help
			// by the horizontal distance from the COM to the planted feet (segment between them, or the one foot).
			float overFeet = 0.0f;
			if (!stance.empty()) {
				const Vec3 c{ com.x, com.y, 0.0f };
				const Vec3 a{ stance.front()->pos.x, stance.front()->pos.y, 0.0f };
				const Vec3 b{ stance.back()->pos.x, stance.back()->pos.y, 0.0f };
				const auto ab = Sub(b, a);
				const float abLen2 = Dot(ab, ab);
				const float t = abLen2 > 1.0e-6f ? std::clamp(Dot(Sub(c, a), ab) / abLen2, 0.0f, 1.0f) : 0.0f;
				const float distance = Length(Sub(c, Add(a, Scale(ab, t))));
				overFeet = std::clamp(1.0f - (distance - kSupportBase) / kSupportFade, 0.0f, 1.0f);
				overFeetLog = overFeet;
			}
			std::vector<std::tuple<const Sample*, Vec3, float>> pushes;  // body, velocity change, share of dOmega it turns by
			Vec3 total{};
			if (catchable > 0.0f && overFeet > 0.0f) {
				for (const auto& s : samples) {
					if (s.ref->id.m_value == a_stumble.stepFoot) continue;  // the swing foot is driven above
					if (std::find(stance.begin(), stance.end(), &s) != stance.end()) continue;  // takes the reaction
					// Weight support goes to every body (supporting only the torso made it hover over the legs);
					// the horizontal/tilt part skips the legs, which stay planted by friction instead of being dragged.
					Vec3 deltaV{ 0.0f, 0.0f, dCom.z };
					float turn = 0.0f;
					if (!s.ref->leg) {
						const auto r = Sub(s.pos, feet);
						const auto spin = Cross(dOmega, r);
						deltaV = Add(dCom, spin);
						// Trunk only: dOmega is measured on the trunk; loose limbs (owned by grip/brace/flail) twitched.
						turn = s.ref->pelvis || s.ref->spine ? 1.0f : 0.0f;
					}
					const float n = Length(deltaV), cap = kBalanceMaxAccel * dt;
					if (!std::isfinite(n) || n < 1.0e-4f) continue;
					if (n > cap) {
						deltaV = Scale(deltaV, cap / n);
						turn *= cap / n;
					}
					deltaV = Scale(deltaV, overFeet);
					pushes.push_back({ &s, deltaV, turn * overFeet });
					total = Add(total, Scale(deltaV, 1.0f / s.motion.inverseMass));
				}
			}
			// Friction cone: sideways help at most kFootFriction × the weight pressed into the ground.
			const float sideways = std::sqrt(total.x * total.x + total.y * total.y);
			const float allowed = kFootFriction * std::max(total.z, 0.0f);
			const float sideScale = sideways > allowed && sideways > 1.0e-6f ? allowed / sideways : 1.0f;
			Vec3 reaction{};
			for (auto& [s, deltaV, turn] : pushes) {
				deltaV.x *= sideScale;
				deltaV.y *= sideScale;
				const auto impulse = Scale(deltaV, 1.0f / s->motion.inverseMass);
				reaction = Sub(reaction, impulse);
				RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
				g_linearImpulse(npWorld, s->ref->id.m_value, &nativeImpulse);
				++pushed;
				// A rigid rotation also turns the trunk bodies about their own centres; the linear part above only
				// carries them around the arc. No reaction for this part: it is small next to the arc (own inertia vs m*r^2).
				if (turn > 0.0f && g_angularImpulse) {
					float inverse[3][3]{};
					AddWorldInverse(s->motion, inverse);
					Vec3 angular{};
					if (Solve3(inverse, Scale(dOmega, turn), angular)) {
						RE::hkVector4f angularImpulse{ angular.x, angular.y, angular.z, 0.0f };
						g_angularImpulse(npWorld, s->ref->id.m_value, &angularImpulse);
					}
				}
			}
			// Sideways reaction only: pushing the light feet down every frame punched them through thin terrain
			// collision (exterior heightfields). The ground supplies the upward part anyway.
			reaction.z = 0.0f;
			for (const auto* foot : stance) {
				const auto impulse = Scale(reaction, 1.0f / static_cast<float>(stance.size()));
				RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
				g_linearImpulse(npWorld, foot->ref->id.m_value, &nativeImpulse);
			}
			ApplyKillingPush(a_stumble, npWorld, samples, com, mass, kDeathPushScale);
		}

		// 3. Grip: a hand clutches the body part the killing shot hit; the free hands brace for the fall.
		if (a_stumble.grip) ApplyGrip(a_stumble, npWorld, samples, dt, 1.0f);
		a_stumble.braceAmount = 0.0f;
		a_stumble.braceOff = -1.0f;
		// Called with lean 0 too, so the brace eases out instead of cutting off at kBraceGround.
		if (kDeathBrace > 0.0f) ApplyBrace(a_stumble, npWorld, samples, dt, braceLean, 1.0f - std::min(flailAmount, 1.0f), groundSurface);  // flailing wins the arms

		// Knee flexion axes from the death pose: cross(thigh->calf, calf->foot) bends the knee further the way it
		// already bends. A straight leg borrows the other knee's axis (both flex about the same lateral axis).
		if (!a_stumble.kneeAxesDone && !a_stumble.joints.empty() && kDeathKneeBend > 0.0f) {
			a_stumble.kneeAxesDone = true;
			std::vector<std::pair<const BodyRef*, Vec3>> knees;  // knee joint, flexion axis (world, death pose)
			for (const auto& joint : a_stumble.joints) {
				if (!joint.knee) continue;
				const auto* calf = findTarget(joint.id.m_value);
				const auto* thigh = findTarget(joint.parentId.m_value);
				const BodyTarget* foot = nullptr;
				for (const auto& body : a_stumble.bodies)
					if (body.foot && body.side == joint.side) foot = findTarget(body.id.m_value);
				Vec3 axis{};
				if (calf && thigh && foot) {
					const auto c = Cross(Unit(Sub(calf->pos, thigh->pos)), Unit(Sub(foot->pos, calf->pos)));
					if (Length(c) > 0.05f) axis = Unit(c);
				}
				knees.push_back({ &joint, axis });
			}
			for (auto& [joint, axis] : knees) {
				if (Length(axis) < 0.5f) {
					for (const auto& [other, otherAxis] : knees)
						if (other != joint && Length(otherAxis) > 0.5f) axis = otherAxis;
				}
				const auto* thigh = findTarget(joint->parentId.m_value);
				if (Length(axis) > 0.5f && thigh) {
					a_stumble.kneeAxes.push_back({ joint->id.m_value, Rotate(Transpose(thigh->rot), axis) });
				}
			}
			REX::INFO("DynamicRagdollSystem: {:08X} knee buckle on {} knees"sv, a_stumble.formID, a_stumble.kneeAxes.size());
		}

		// 2. Joint servo: paired internal torques restore the captured relative joint rotations.
		std::int32_t twisted = 0;
		const float jointCap = kJointMaxTorque * dt * fade;
		for (const auto& joint : a_stumble.joints) {
			BodyRef parentRef{};
			parentRef.index = joint.parentIndex;
			parentRef.id = joint.parentId;
			const auto* childTarget = findTarget(joint.id.m_value);
			const auto* parentTarget = findTarget(joint.parentId.m_value);
			if (!childTarget || !parentTarget || !valid(joint) || !valid(parentRef)) continue;
			if (joint.arm) continue;  // arms are free (grip, twitches); the servo holds spine and legs
			const bool swingLeg = a_stumble.stepFoot && joint.leg && joint.side == a_stumble.stepSide;
			if (swingLeg && !joint.knee) continue;  // let the swing hip move freely; its knee is driven to fold
			auto& child = worldBodies[joint.index];
			auto& parent = worldBodies[joint.parentIndex];
			if (!ConstraintCurrent(npWorld, joint.constraintId, child.m_id.m_value, parent.m_id.m_value)) continue;

			MotionState childMotion{}, parentMotion{};
			if (!ReadMotionState(npWorld, child, childMotion) || !ReadMotionState(npWorld, parent, parentMotion)) continue;
			const auto parentRotation = RotationFrom(parent.m_transform.m_rotation);
			const auto currentRelative = RelativeRotation(parentRotation, RotationFrom(child.m_transform.m_rotation));
			auto targetRelative = RelativeRotation(parentTarget->rot, childTarget->rot);
			if (joint.knee) {
				// Knees fold progressively as the body weakens, instead of holding the death pose (which left a
				// leg that was reaching forward mid-stride stretched out).
				for (const auto& [id, axis] : a_stumble.kneeAxes) {
					if (id != joint.id.m_value) continue;
					// Full bend by two thirds of the stumble; the stepping knee folds extra mid-swing.
					float bend = kDeathKneeBend * std::min(1.0f, 1.5f * (1.0f - fade));
					if (swingLeg) {
						const float phase = std::clamp((a_now - a_stumble.stepStart) / std::max(a_stumble.stepTime, 0.01f), 0.0f, 1.0f);
						bend += kSwingKneeBend * std::sin(3.14159265f * phase);
					}
					targetRelative = Multiply(AxisAngle(axis, bend * 0.0174533f), targetRelative);
				}
			}
			const auto errorWorld = Rotate(parentRotation, RotationError(currentRelative, targetRelative));
			const auto relativeOmega = Sub(Rotate(childMotion.orientation, childMotion.angular),
				Rotate(parentMotion.orientation, parentMotion.angular));
			// Knees get a firmer, less faded drive: at low stiffness a loaded leg never reached its bend.
			const float stiffness = joint.knee ? std::max(kDeathStiffness, kKneeMinStiffness) : kDeathStiffness;
			// Knee floor ramps out over the last 10% of the stumble instead of snapping from kKneeMinFade to 0 at the end.
			const float jointFade = joint.knee ? std::max(fade, std::min(kKneeMinFade, 10.0f * fade)) : fade;
			Vec3 desired = Scale(Add(Scale(errorWorld, -kJointKp * stiffness), Scale(relativeOmega, -kJointKd * stiffness)), dt * jointFade);
			const float desiredNorm = Length(desired);
			if (!std::isfinite(desiredNorm) || desiredNorm < 0.002f) continue;
			if (desiredNorm > kJointMaxDeltaOmega) desired = Scale(desired, kJointMaxDeltaOmega / desiredNorm);
			if (ApplyJointImpulse(npWorld, joint, parentRef, desired, joint.knee ? kJointMaxTorque * dt * jointFade : jointCap)) ++twisted;
		}

		// 4. Flail: a body flying through the air swings its arms and legs (paired joint impulses = muscles, so
		//    the flight path is unchanged) instead of being dragged along limp. Each limb joint follows its own
		//    random axis and rhythm; the arm clutching the wound is left alone.
		if (flailAmount > 0.0f && a_stumble.flail > 0.0f) {
			char gripSide = 0;
			if (a_stumble.grip)
				for (const auto& body : a_stumble.bodies)
					if (body.id.m_value == a_stumble.gripHand) gripSide = body.side;
			// no fade: a flight outlasting the stumble timer still flails
			const auto flailing = FlailLimbs(npWorld, a_stumble, a_now - a_stumble.start, dt, flailAmount, a_stumble.flail * flailAmount,
				gripSide, a_stumble.stepFoot ? a_stumble.stepSide : 0);
			if (!a_stumble.flailLogged && flailing > 0) {
				a_stumble.flailLogged = true;
				REX::INFO("DynamicRagdollSystem: {:08X} flailing ({} limb joints, amount {:.2f})"sv, a_stumble.formID, flailing, flailAmount);
			}
		}

		if (++a_stumble.balanceSteps == 1 || a_stumble.balanceSteps % 30 == 0) {
			REX::INFO("DynamicRagdollSystem: {:08X} death balance {} pushed {}/{} (not dynamic {}, no motion {}) joints {}/{} tilt {:.2f} grounded {} fade {:.2f}"sv, a_stumble.formID,
				a_stumble.balanceSteps, pushed, a_stumble.bodies.size(), notDynamic, noMotion, twisted, a_stumble.joints.size(),
				Length(tiltError), grounded, fade);
			REX::INFO("DynamicRagdollSystem: {:08X}   flail {:.2f} brace {:.2f} (off {:.2f}) height {:.2f} overFeet {:.2f} speed {:.2f} lean {:.2f} help {:.2f}"sv, a_stumble.formID,
				flailAmount, a_stumble.braceAmount, a_stumble.braceOff, com.z - trunkLevel, overFeetLog,
				std::sqrt(comVel.x * comVel.x + comVel.y * comVel.y), Length(tiltError),
				std::clamp(1.0f - (std::sqrt(comVel.x * comVel.x + comVel.y * comVel.y) - kCatchSpeed) / kCatchRange, 0.0f, 1.0f) *
					std::clamp((kGiveUpTilt - Length(tiltError)) / kGiveUpRange, 0.0f, 1.0f));
		}
	}
}
