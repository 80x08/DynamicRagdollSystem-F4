#include "Common.h"

namespace DynamicRagdollSystem
{

	// A hand clutches the body part the killing shot hit. Equal and opposite impulses on hand/arm and wound,
	// so it acts like the arm's own muscles (no outside force). a_strength 0..1 lets the grip loosen.
	void ApplyGrip(Death& a_stumble, RE::hknpBSWorld* npWorld, const std::vector<DeathSample>& samples, float dt, float a_strength)
	{
		// 3. Grip: a hand clutches the body part the killing shot hit. Equal and opposite impulses on hand and
		//    wound, so it acts like the arm's own muscles (no outside force) and keeps holding while it falls.
		if (a_stumble.hitBody != 0x7FFFFFFF && a_strength > 0.0f) {
			const auto find = [&](std::uint32_t a_id) -> const DeathSample* {
				for (const auto& s : samples)
					if (s.ref->id.m_value == a_id) return &s;
				return nullptr;
			};
			// A dismembered wound: clutch the stump (nearest still-attached ancestor) instead of the cut-off part.
			std::uint32_t woundId = a_stumble.hitBody;
			for (int depth = 0; depth < 8 && a_stumble.Detached(woundId); ++depth) {
				const auto it = std::find_if(a_stumble.bodies.begin(), a_stumble.bodies.end(),
					[&](const BodyRef& a_b) { return a_b.id.m_value == woundId; });
				if (it == a_stumble.bodies.end()) break;
				woundId = it->parentId.m_value;
			}
			const DeathSample* wound = find(woundId);
			if (wound && !a_stumble.gripChosen) {
				a_stumble.gripChosen = true;
				float best = std::numeric_limits<float>::max();
				for (const auto& s : samples) {
					if (!s.ref->hand || s.ref->id.m_value == woundId) continue;
					const float d = Length(Sub(s.pos, wound->pos));
					if (d < best) {
						best = d;
						a_stumble.gripHand = s.ref->id.m_value;
					}
				}
				REX::INFO("DynamicRagdollSystem: {:08X} grip: {} hand clutches hit body {}"sv, a_stumble.formID,
					a_stumble.gripHand ? "a" : "no", a_stumble.hitBody);
			}
			if (const DeathSample* hand = a_stumble.gripHand ? find(a_stumble.gripHand) : nullptr; hand && wound) {
				// The hand alone is light: the joints spread its pull over the heavier forearm and upper arm and
				// gravity wins. Size the pull for the whole arm and carry the arm's weight while it clutches.
				float armMass = 0.0f;
				const auto& g = npWorld->m_gravity;
				Vec3 woundImpulse{};
				for (const auto& s : samples) {
					if (!s.ref->arm || s.ref->side != hand->ref->side) continue;
					const float m = 1.0f / s.motion.inverseMass;
					armMass += m;
					const auto lift = Scale(Vec3{ g.x, g.y, g.z }, -m * dt * a_strength);
					RE::hkVector4f liftImpulse{ lift.x, lift.y, lift.z, 0.0f };
					g_linearImpulse(npWorld, s.ref->id.m_value, &liftImpulse);
					woundImpulse = Sub(woundImpulse, lift);
				}
				const auto relVel = Sub(hand->motion.linear, wound->motion.linear);
				Vec3 deltaV = Scale(Sub(Scale(Sub(wound->pos, hand->pos), kGripKp), Scale(relVel, kGripKd)), dt * a_strength);
				const float n = Length(deltaV), cap = kGripMaxAccel * dt;
				if (std::isfinite(n) && n > 1.0e-4f) {
					if (n > cap) deltaV = Scale(deltaV, cap / n);
					const auto impulse = Scale(deltaV, std::max(armMass, 1.0f / hand->motion.inverseMass));
					RE::hkVector4f toHand{ impulse.x, impulse.y, impulse.z, 0.0f };
					g_linearImpulse(npWorld, a_stumble.gripHand, &toHand);
					woundImpulse = Sub(woundImpulse, impulse);
				}
				RE::hkVector4f toWound{ woundImpulse.x, woundImpulse.y, woundImpulse.z, 0.0f };  // equal and opposite
				g_linearImpulse(npWorld, woundId, &toWound);
			}
		}

	}


	// Ground height under a point (physics units), via the engine's pick ray (bhkPickData + TESObjectCELL::Pick;
	// start/end in game units, the pick scales them). Returns false when nothing is hit within kRayDown.
	bool CastGround(RE::TESObjectCELL* a_cell, Vec3 a_pos, float& a_ground)
	{
		constexpr float kGame = 69.99125f;  // game units per physics unit
		RE::bhkPickData pick{};
		const RE::NiPoint3 from{ a_pos.x * kGame, a_pos.y * kGame, (a_pos.z + kRayUp) * kGame };
		const RE::NiPoint3 to{ a_pos.x * kGame, a_pos.y * kGame, (a_pos.z - kRayDown) * kGame };
		pick.SetStartEnd(from, to);
		pick.castQuery.m_filterData.m_collisionFilterInfo = static_cast<std::uint32_t>(RE::COL_LAYER::kPathingPick);
		// Explicit layer list (bhkWorld::Pick then uses hknpBSCustomCollisionFilter: one bit per layer; bodies with a
		// collision group, like ragdoll parts, are only hit if their layer is listed; ungrouped world geometry always).
		using L = RE::COL_LAYER;
		constexpr auto bit = [](L a_layer) { return std::uint64_t{ 1 } << static_cast<std::uint32_t>(a_layer); };
		pick.customCollideLayers = bit(L::kStatic) | bit(L::kAnimStatic) | bit(L::kTrees) | bit(L::kProps) | bit(L::kTerrain) |
		                           bit(L::kGround) | bit(L::kStairHelper);
		static_cast<void>(a_cell->Pick(pick));  // the hit is read from pick, not the returned object
		if (!pick.HasHit()) return false;
		const float fraction = pick.GetHitFraction();
		if (!std::isfinite(fraction)) return false;
		a_ground = a_pos.z + kRayUp - (kRayUp + kRayDown) * fraction;
		return true;
	}


	// Rays under both feet and the trunk, cast outside the world lock (a pick takes the world's read lock).
	void UpdateGround(Death& a_death, RE::Actor* a_actor, RE::bhkWorld* a_world)
	{
		a_death.groundValid = false;
		auto* npWorld = a_world ? a_world->m_worldNP.get() : nullptr;
		auto* cell = a_actor->GetParentCell();
		if (!npWorld || !cell || a_death.bodies.empty() || a_death.targets.empty()) return;
		auto& worldBodies = npWorld->m_bodyManager.m_bodies;
		const BodyRef* trunk = nullptr;
		int feet = 0;
		bool hitAny = false;
		for (const auto& body : a_death.bodies) {
			if (body.index < 0 || body.index >= worldBodies.size() || worldBodies[body.index].m_id.m_value != body.id.m_value) continue;
			if (a_death.Detached(body.id.m_value)) continue;
			if (!trunk || body.priority < trunk->priority) trunk = &body;
			if (!body.foot || feet >= 2) continue;
			const auto pos = BodyPosition(worldBodies[body.index]);
			float ground = pos.z - kRayDown;
			hitAny |= CastGround(cell, pos, ground);
			a_death.groundFootId[feet] = body.id.m_value;
			a_death.groundFoot[feet] = ground;
			++feet;
		}
		a_death.groundFeet = feet;
		// Landing spot of the current step (stairs, slopes), once per step.
		if (a_death.stepFoot && !a_death.stepGroundTried) {
			a_death.stepGroundTried = true;
			const auto& to = a_death.stepTo;
			a_death.stepGroundHit = CastGround(cell, Vec3{ to.x, to.y, to.z + kStepRayLift }, a_death.stepGround);
			if (a_death.groundCaptured)
				REX::INFO("DynamicRagdollSystem: {:08X} death step {} landing ground {}"sv, a_death.formID, a_death.steps,
					a_death.stepGroundHit ? std::format("{:+.2f} from the ground at death", a_death.stepGround - a_death.groundAtDeath) : "not found"s);
		}
		if (feet == 0 || !hitAny || !trunk) return;
		const auto trunkPos = BodyPosition(worldBodies[trunk->index]);
		a_death.groundTrunk = trunkPos.z - kRayDown;
		CastGround(cell, trunkPos, a_death.groundTrunk);
		if (!a_death.groundCaptured) {
			a_death.groundCaptured = true;
			a_death.groundAtDeath = feet == 2 ? 0.5f * (a_death.groundFoot[0] + a_death.groundFoot[1]) : a_death.groundFoot[0];
			REX::INFO("DynamicRagdollSystem: {:08X} ground ray: feet {:.2f} above the ground at death"sv, a_death.formID,
				a_death.feetCaptured.z - a_death.groundAtDeath);
		}
		a_death.groundValid = true;
	}


	// Brace: when the body tips over, the free hands (not the one clutching the wound) reach for the ground ahead
	// of the chest in the fall direction. Equal and opposite impulses on hand/arm and chest, like the grip.
	void ApplyBrace(Death& a_death, RE::hknpBSWorld* npWorld, const std::vector<DeathSample>& samples, float dt, float a_lean, float a_scale,
		float a_ground)
	{
		const float amount = std::clamp((a_lean - kBraceLean) / (kBraceFull - kBraceLean), 0.0f, 1.0f) * kDeathBrace * a_scale;
		if (amount <= 0.0f) return;
		const auto [low, high] = TrunkEnds(samples);  // pelvis / SPINE1, chest
		if (!low || !high || low == high) return;
		// Fall direction = where the trunk leans; nearly upright, where the chest is moving.
		Vec3 fall{ high->pos.x - low->pos.x, high->pos.y - low->pos.y, 0.0f };
		if (Length(fall) < 0.05f) fall = Vec3{ high->motion.linear.x, high->motion.linear.y, 0.0f };
		if (Length(fall) < 1.0e-3f) return;
		fall = Unit(fall);
		// Head guard in the trunk's frame (stays the same shape as the body tips). Reaching straight out read as a
		// stiff outstretched arm, not a brace.
		const auto up = Unit(Sub(high->pos, low->pos));
		auto forward = Sub(fall, Scale(up, Dot(fall, up)));
		forward = Length(forward) > 0.05f ? Unit(forward) : fall;
		const auto right = Cross(forward, up);
		const auto guardPoint = [&](float a_up, float a_forward, float a_side) {
			auto p = Add(Add(high->pos, Scale(up, a_up)), Add(Scale(forward, a_forward), Scale(right, a_side)));
			p.z = std::max(p.z, a_ground + 0.05f);
			return p;
		};
		const auto& g = npWorld->m_gravity;
		a_death.braceAmount = amount;
		float offSum = 0.0f;
		int offCount = 0;
		Vec3 chestImpulse{};
		for (const auto& hand : samples) {
			if (!hand.ref->hand || hand.ref->id.m_value == a_death.gripHand) continue;
			const float sideSign = hand.ref->side == 'L' ? -1.0f : 1.0f;
			const auto target = guardPoint(kGuardHandUp, kGuardHandForward, kGuardHandSide * sideSign);
			const DeathSample* forearm = nullptr;  // the hand's parent body: drives the elbow forward
			for (const auto& s : samples)
				if (s.ref->id.m_value == hand.ref->parentId.m_value && s.ref->arm) forearm = &s;
			// Carry the arm's weight while it reaches (otherwise it just sags), paid by the chest like a muscle.
			float armMass = 0.0f;
			const float carry = std::min(amount, 1.0f);
			for (const auto& s : samples) {
				if (!s.ref->arm || s.ref->side != hand.ref->side) continue;
				const float m = 1.0f / s.motion.inverseMass;
				armMass += m;
				const auto lift = Scale(Vec3{ g.x, g.y, g.z }, -m * dt * carry);
				RE::hkVector4f liftImpulse{ lift.x, lift.y, lift.z, 0.0f };
				g_linearImpulse(npWorld, s.ref->id.m_value, &liftImpulse);
				chestImpulse = Sub(chestImpulse, lift);
			}
			offSum += Length(Sub(target, hand.pos));
			++offCount;
			// Hand and forearm each pulled to their guard point, the arm's mass split between them.
			const auto drive = [&](const DeathSample& a_body, Vec3 a_target, float a_mass) {
				const auto relVel = Sub(a_body.motion.linear, high->motion.linear);
				Vec3 deltaV = Scale(Sub(Scale(Sub(a_target, a_body.pos), kBraceKp), Scale(relVel, kBraceKd)), dt * amount);
				const float n = Length(deltaV), cap = kBraceMaxAccel * dt;
				if (!std::isfinite(n) || n < 1.0e-4f) return;
				if (n > cap) deltaV = Scale(deltaV, cap / n);
				const auto impulse = Scale(deltaV, std::max(a_mass, 1.0f / a_body.motion.inverseMass));
				RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
				g_linearImpulse(npWorld, a_body.ref->id.m_value, &nativeImpulse);
				chestImpulse = Sub(chestImpulse, impulse);
			};
			drive(hand, target, forearm ? 0.5f * armMass : armMass);
			if (forearm) drive(*forearm, guardPoint(kGuardElbowUp, kGuardElbowForward, kGuardElbowSide * sideSign), 0.5f * armMass);
		}
		RE::hkVector4f toChest{ chestImpulse.x, chestImpulse.y, chestImpulse.z, 0.0f };
		g_linearImpulse(npWorld, high->ref->id.m_value, &toChest);
		a_death.braceOff = offCount ? offSum / static_cast<float>(offCount) : -1.0f;
		if (!a_death.braceLogged) {
			a_death.braceLogged = true;
			REX::INFO("DynamicRagdollSystem: {:08X} brace: arms guard the head (lean {:.2f}, strength {:.2f})"sv, a_death.formID, a_lean, amount);
		}
	}


	void StepDeath(Death& a_stumble, RE::Actor* a_actor, float a_now)
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
			return a_ref.index >= 0 && a_ref.index < worldBodies.size() && worldBodies[a_ref.index].m_id.m_value == a_ref.id.m_value &&
			       !a_stumble.Detached(a_ref.id.m_value);
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
				a_stumble.targets.push_back({ body.id.m_value, RotationFrom(b.m_transform.m_rotation), BodyPosition(b) });
			}
			a_stumble.hasFeet = feetMidpoint(a_stumble.feetCaptured);
			REX::INFO("DynamicRagdollSystem: {:08X} death pose captured after {:.2f}s ({} bodies, feet {}, gravity z {:.2f})"sv, a_stumble.formID,
				a_now - a_stumble.start, a_stumble.targets.size(), a_stumble.hasFeet, npWorld->m_gravity.z);
		}
		if (a_stumble.targets.empty()) return;
		UpdateDetached(a_stumble, npWorld);
		if (!a_stumble.stumble) {
			a_stumble.end = a_now;  // plain ragdoll fall; grip/twitch run in the dying phase
			return;
		}

		const auto findTarget = [&](std::uint32_t a_id) -> const BodyTarget* {
			for (const auto& target : a_stumble.targets)
				if (target.id == a_id) return &target;
			return nullptr;
		};

		const float dt = a_stumble.lastStep > 0.0f ? std::clamp(a_now - a_stumble.lastStep, 0.001f, 0.05f) : 1.0f / 60.0f;
		a_stumble.lastStep = a_now;
		// Strength decays steadily from the moment of death, so the body buckles progressively instead of
		// standing at full strength and dropping at the end.
		const float fade = std::clamp((a_stumble.end - a_now) / std::max(a_stumble.end - a_stumble.start, 0.01f), 0.0f, 1.0f);
		if (fade <= 0.0f) return;

		// 1. Whole-rig balance. Every body gets the velocity change of one rigid body, so the
		//    whole mass is moved (no per-body pinning, which froze the rig into a statue):
		//    - tilt: rotate about the feet toward the captured trunk tilt (yaw left free),
		//    - height: spring the centre of mass to its captured height above the feet, cancel gravity,
		//    - horizontal: only light damping, so the hit's momentum turns into a stumble.
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
		float overFeetLog = -1.0f; // ledge check (COM over the planted feet), for the log; -1 = no planted foot
		if (mass > 0.0f && trunkTarget && trunk->priority < 99 && feetMidpoint(feet)) {
			com = Scale(com, 1.0f / mass);
			comVel = Scale(comVel, 1.0f / mass);
			// Weapon knockback: guns with a big impact force launch the corpse faster than it can be caught. For a
			// short window, speed beyond the pre-death movement (sideways, or upward) is cut to fDeathKnockbackScale
			// of the largest seen. Tracking the peak keeps it from compounding frame to frame. Not for explosions.
			if (a_now - a_stumble.start < kKnockWindow && !a_stumble.flung && kDeathKnockbackScale < 1.0f) {
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
			// Flail while it cannot catch itself (too fast) or it is off the ground, until it lands. Off the ground =
			// feet up *and* the COM at least near standing height: buckling knees fold the feet up in a collapse.
			if (a_stumble.standHeight <= 0.0f) a_stumble.standHeight = com.z - trunkLevel;
			const bool airborne = feet.z - feetLevel >= kGroundTolerance && com.z - trunkLevel > 0.9f * a_stumble.standHeight;
			// Height above the ground it stood on (not above its own feet: a tumbling body has its feet level with it).
			const bool landed = com.z - trunkLevel < kFlailGround;
			const float fast = std::clamp((speed - kCatchSpeed) / std::max(kCatchRange, 0.1f), 0.0f, 1.0f);
			// Falling (e.g. off a ledge, feet below the captured ground) also flails, until the drop slows down.
			// Only below the ground it stood on, so an ordinary collapse to the floor does not flail on the way down.
			const bool belowGround = com.z - trunkLevel < -0.3f;
			const float falling = belowGround ? std::clamp((-comVel.z - 1.5f) / 1.5f, 0.0f, 1.0f) : 0.0f;
			flailAmount = std::max(landed ? 0.0f : std::max(fast, airborne ? 1.0f : 0.0f), falling);
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
				if (swing == samples.end() || phase >= 1.0f) {
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
			// One-off push from the killing hit: the hit body part gets kHitPush, the rest of the upper body
			// kBodyPush (whole upper body kHitPush if the hit part is unknown). No push without a recent hit.
			if (!a_stumble.hitApplied) {
				a_stumble.hitApplied = true;
				const bool hitKnown = std::any_of(samples.begin(), samples.end(),
					[&](const Sample& a_s) { return a_s.ref->id.m_value == a_stumble.hitBody; });
				Vec3 ownPush{};
				for (const auto& s : samples) {
					const bool isHit = s.ref->id.m_value == a_stumble.hitBody;
					if (!isHit && s.pos.z <= com.z) continue;
					auto impulse = Scale(a_stumble.hitDir, (isHit || !hitKnown ? kHitPush : kBodyPush) * kDeathPushScale / s.motion.inverseMass);
					if (s.ref->foot || s.ref->knee) impulse.z = std::max(impulse.z, 0.0f);  // floor-level bodies: not into the ground
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
		}

		// 3. Grip: a hand clutches the body part the killing shot hit; the free hands brace for the fall.
		if (a_stumble.grip) ApplyGrip(a_stumble, npWorld, samples, dt, 1.0f);
		a_stumble.braceAmount = 0.0f;
		a_stumble.braceOff = -1.0f;
		if (kDeathBrace > 0.0f && braceLean > 0.0f) ApplyBrace(a_stumble, npWorld, samples, dt, braceLean, 1.0f - std::min(flailAmount, 1.0f), groundSurface);  // flailing wins the arms

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
			const float jointFade = joint.knee ? std::max(fade, kKneeMinFade) : fade;
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



	// Living NPC in an engine ragdoll (explosion knockdown, push, fall): flying, the limbs flail; otherwise the free
	// hands brace as it tips over. The death brace and flail, without any balance help.
	void StepKnockdown(Death& a_down, RE::Actor* a_actor, float a_now)
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
			if (body.index < 0 || body.index >= worldBodies.size() || worldBodies[body.index].m_id.m_value != body.id.m_value) continue;
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
		const float dt = a_down.lastStep > 0.0f ? std::clamp(a_now - a_down.lastStep, 0.001f, 0.05f) : 1.0f / 60.0f;
		a_down.lastStep = a_now;
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


	// After the stumble: the body lies there clutching the wound, twitching and now and then rolling, with
	// everything decaying to fully limp over kDyingTime. Twitches are paired joint impulses (spine/legs; the
	// gripping arm is not in the joint list); rolls are a small sideways kick on the trunk.
	void StepDying(Death& a_death, RE::Actor* a_actor, float a_now)
	{
		auto* world = GetWorld(a_actor);
		auto* npWorld = world ? world->m_worldNP.get() : nullptr;
		if (!npWorld || !g_linearImpulse || a_death.bodies.empty()) return;
		WorldWriteLock lock{ npWorld };
		if (!lock.held) return;
		auto& worldBodies = npWorld->m_bodyManager.m_bodies;
		std::vector<DeathSample> samples;
		UpdateDetached(a_death, npWorld);
		// Head cut off during the dying phase ends it (OnWorldUpdate erases the entry); already off at the start: carry on.
		const bool headOff = std::any_of(a_death.bodies.begin(), a_death.bodies.end(),
			[&](const BodyRef& a_body) { return a_body.head && a_death.Detached(a_body.id.m_value); });
		if (!a_death.dyingStepped) {
			a_death.dyingStepped = true;
			a_death.headOffBefore = headOff;
		} else if (kDyingEndOnHeadLoss && headOff && !a_death.headOffBefore) {
			a_death.headCut = true;
			return;
		}
		for (const auto& body : a_death.bodies) {
			if (body.index < 0 || body.index >= worldBodies.size() || worldBodies[body.index].m_id.m_value != body.id.m_value) continue;
			if (a_death.Detached(body.id.m_value)) continue;
			MotionState motion{};
			if (!ReadMotionState(npWorld, worldBodies[body.index], motion)) continue;
			samples.push_back({ &body, motion, BodyPosition(worldBodies[body.index]) });
		}
		if (samples.empty()) return;
		const float dt = a_death.lastStep > 0.0f ? std::clamp(a_now - a_death.lastStep, 0.001f, 0.05f) : 1.0f / 60.0f;
		a_death.lastStep = a_now;
		// The dying clock starts once the trunk is lying (or kDownTimeout after the stumble). Starting it at the
		// end of the stumble spent the strongest twitches and rolls while the body was still falling.
		if (a_death.downTime <= 0.0f) {
			const auto [low, high] = TrunkEnds(samples);  // pelvis / SPINE1, chest
			// Down = trunk near horizontal and the body has nearly stopped (it can be flat while still tumbling).
			Vec3 momentum{};
			float mass = 0.0f;
			for (const auto& sample : samples) {
				momentum = Add(momentum, Scale(sample.motion.linear, 1.0f / sample.motion.inverseMass));
				mass += 1.0f / sample.motion.inverseMass;
			}
			const float comSpeed = mass > 0.0f ? Length(momentum) / mass : 0.0f;
			const bool lying = low && high && low != high && std::abs(Unit(Sub(high->pos, low->pos)).z) < 0.5f && comSpeed < 0.6f;
			if (lying || a_now >= a_death.end + kDownTimeout) {
				a_death.downTime = a_now;
				a_death.nextRoll = a_now + kFirstRollDelay;
				a_death.dyingLogged = true;
				REX::INFO("DynamicRagdollSystem: {:08X} dying phase ({:.1f}s), body {} {:.2f}s after the stumble"sv, a_death.formID,
					a_death.dyingTime, lying ? "down" : "still up", a_now - a_death.end);
			} else {
				if (a_death.grip) ApplyGrip(a_death, npWorld, samples, dt, 1.0f);  // keep clutching while it falls
				// The body usually goes down right as the stumble ends: brace here too, with the lean from vertical.
				if (kDeathBrace > 0.0f && low && high && low != high) {
					const float lean = std::acos(std::clamp(Unit(Sub(high->pos, low->pos)).z, -1.0f, 1.0f));
					const float ground = a_death.groundValid ? a_death.groundTrunk : a_death.feetCaptured.z - 0.1f;
					// Only while still falling: a body already lying but sliding (not "down" yet) reads as fully leaning.
					if (low->pos.z - ground > kBraceGround + 0.05f) ApplyBrace(a_death, npWorld, samples, dt, lean, 1.0f, ground);
				}
				return;
			}
		}
		const float progress = std::clamp((a_now - a_death.downTime) / std::max(a_death.dyingTime, 0.01f), 0.0f, 1.0f);
		const float strength = 1.0f - progress;

		// The grip holds fully for the first half, then loosens.
		if (a_death.grip) ApplyGrip(a_death, npWorld, samples, dt, std::min(1.0f, 2.0f * strength));

		static std::mt19937 rng{ std::random_device{}() };
		std::uniform_real_distribution<float> unit{ 0.0f, 1.0f };
		const float twitch = a_death.twitch * strength;
		// A twitch is a short burst: the same joint is driven the same way for kTwitchBurst. A single-frame kick
		// was eaten by joint friction and ground contact within a frame or two.
		if (twitch > 0.0f && a_now >= a_death.nextTwitch && !a_death.joints.empty() && g_angularImpulse) {
			a_death.nextTwitch = a_now + (0.15f + 0.35f * unit(rng)) * (1.0f + 1.5f * progress);  // spasms thin out
			// Any spine/leg/arm joint except the arm that clutches the wound.
			char gripSide = 0;
			for (const auto& sample : samples)
				if (sample.ref->id.m_value == a_death.gripHand) gripSide = sample.ref->side;
			a_death.twitchJoint = -1;
			for (int attempt = 0; attempt < 6 && a_death.twitchJoint < 0; ++attempt) {
				const auto pick = static_cast<std::int32_t>(rng() % a_death.joints.size());
				const auto& candidate = a_death.joints[static_cast<std::size_t>(pick)];
				if (!(candidate.arm && gripSide && candidate.side == gripSide) && !a_death.Detached(candidate.id.m_value)) a_death.twitchJoint = pick;
			}
			a_death.twitchAxis = Unit(Vec3{ unit(rng) * 2.0f - 1.0f, unit(rng) * 2.0f - 1.0f, unit(rng) * 2.0f - 1.0f });
			// Light limbs (calves, feet, hands) twitch less: same speed change on a light part flung it high.
			a_death.twitchScale = 1.0f;
			if (a_death.twitchJoint >= 0) {
				const auto& picked = a_death.joints[static_cast<std::size_t>(a_death.twitchJoint)];
				float heaviest = 0.0f, mass = 0.0f;
				for (const auto& sample : samples) {
					const float m = 1.0f / sample.motion.inverseMass;
					for (const auto& joint : a_death.joints)
						if (joint.id.m_value == sample.ref->id.m_value) heaviest = std::max(heaviest, m);
					if (sample.ref->id.m_value == picked.id.m_value) mass = m;
				}
				if (heaviest > 0.0f && mass > 0.0f) a_death.twitchScale = std::clamp(mass / heaviest, kTwitchMinScale, 1.0f);
			}
			a_death.twitchEnd = a_now + kTwitchBurst;
			++a_death.twitchCount;
			// Jolt: a sharp jerk on the limb, equal and opposite on the part it hangs from (no net shove).
			if (a_death.twitchJoint >= 0) {
				const auto& joint = a_death.joints[static_cast<std::size_t>(a_death.twitchJoint)];
				const DeathSample* child = nullptr;
				const DeathSample* parent = nullptr;
				for (const auto& sample : samples) {
					if (sample.ref->id.m_value == joint.id.m_value) child = &sample;
					if (sample.ref->id.m_value == joint.parentId.m_value) parent = &sample;
				}
				if (child && parent) {
					// Mostly up/sideways for the trunk and arms; legs jerk sideways only, so they do not kick up.
					const float up = joint.leg ? 0.0f : unit(rng) * 1.5f;
					const Vec3 dir = Unit(Vec3{ unit(rng) * 2.0f - 1.0f, unit(rng) * 2.0f - 1.0f, up });
					const auto impulse = Scale(dir, kTwitchJolt * twitch * a_death.twitchScale / child->motion.inverseMass);
					RE::hkVector4f toChild{ impulse.x, impulse.y, impulse.z, 0.0f };
					RE::hkVector4f toParent{ -impulse.x, -impulse.y, -impulse.z, 0.0f };
					g_linearImpulse(npWorld, joint.id.m_value, &toChild);
					g_linearImpulse(npWorld, joint.parentId.m_value, &toParent);
				}
			}
		}
		if (a_death.twitchJoint >= 0 && a_now < a_death.twitchEnd && twitch > 0.0f) {
			const auto& joint = a_death.joints[static_cast<std::size_t>(a_death.twitchJoint)];
			BodyRef parentRef{};
			parentRef.index = joint.parentIndex;
			parentRef.id = joint.parentId;
			if (ApplyJointImpulse(npWorld, joint, parentRef, Scale(a_death.twitchAxis, kTwitchAccel * twitch * a_death.twitchScale * dt),
					kTwitchMaxTorque * twitch * dt)) {
				++a_death.twitchHits;
			}
		}
		// Roll: turn the trunk about its own long axis (pelvis → chest), the axis a lying body rolls around,
		// for a short burst. A one-frame sideways kick only slid the body along the ground.
		const float roll = a_death.roll * std::max(strength, kRollMinStrength);
		if (roll > 0.0f && strength > 0.0f && a_now >= a_death.nextRoll) {
			a_death.nextRoll = a_now + (a_death.burnDown ? kBurnRollGapMin + (kBurnRollGapMax - kBurnRollGapMin) * unit(rng)
			                                             : kRollGap * (1.0f + unit(rng)));
			a_death.rollEnd = a_now + kRollBurst;
			a_death.rollSign = unit(rng) < 0.5f ? -1.0f : 1.0f;
			a_death.rollApplied = false;
		}
		if (roll > 0.0f && a_now < a_death.rollEnd) {
			const auto [low, high] = TrunkEnds(samples);  // pelvis / SPINE1, chest
			if (low && high && low != high) {
				// Whole-body roll: every body gets the velocity of the rig rotating about the trunk's long axis
				// (through the trunk centre). Torquing only the trunk could not turn the limbs hanging off it.
				const auto axis = Unit(Sub(high->pos, low->pos));
				if (std::abs(axis.z) > 0.3f) return;  // not lying flat: rolling about this axis would spin or flip it
				if (!a_death.rollApplied) {
					a_death.rollApplied = true;
					++a_death.rollCount;
				}
				const auto centre = Scale(Add(high->pos, low->pos), 0.5f);
				const auto dOmega = Scale(axis, a_death.rollSign * kRollAccel * roll * dt);
				const float cap = kRollMaxAccel * dt;
				for (const auto& sample : samples) {
					auto r = Sub(sample.pos, centre);
					r = Sub(r, Scale(axis, Dot(r, axis)));  // perpendicular offset from the axis
					auto deltaV = Cross(dOmega, r);
					if (const float n = Length(deltaV); n > cap) deltaV = Scale(deltaV, cap / n);
					// Legs are only dragged along by the hips: no lift and no spin of their own. Limp legs given the
					// full rigid-body roll kept flying up after the burst.
					const bool leg = sample.ref->leg;
					if (leg) deltaV.z = std::min(deltaV.z, 0.0f);
					const auto impulse = Scale(deltaV, 1.0f / sample.motion.inverseMass);
					RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
					g_linearImpulse(npWorld, sample.ref->id.m_value, &nativeImpulse);
					// Each body also turns about the axis itself: the trunk bodies sit on the axis, so the linear
					// part alone moved the limbs while the torso never turned over.
					if (g_angularImpulse && !leg) {
						float inverse[3][3]{};
						AddWorldInverse(sample.motion, inverse);
						Vec3 angular{};
						if (Solve3(inverse, dOmega, angular)) {
							RE::hkVector4f angularImpulse{ angular.x, angular.y, angular.z, 0.0f };
							g_angularImpulse(npWorld, sample.ref->id.m_value, &angularImpulse);
						}
					}
				}
			}
		}
	}


	void Hook_KillImpl(RE::Actor* a_actor, RE::Actor* a_attacker, float a_damage, bool a_sendEvent, bool a_ragdollInstant)
	{
		// Kill moves, sync anims and furniture keep the vanilla death (forcing a ragdoll there cuts paired animations).
		const bool npc = a_actor && !a_actor->IsPlayerRef() && !a_actor->IsDead(false) && !InLockedAnimation(a_actor);
		if (a_actor && !a_actor->IsPlayerRef() && !a_actor->IsDead(false))
			if (const auto why = LockedReason(a_actor)) LogSkipOnce(a_actor, "death skipped", why);
		const bool tracked = npc && (kDeathStumbleChance > 0.0f || kDeathGripChance > 0.0f || kTwitchChance > 0.0f);
		if (npc && kDisableDeathAnimation) a_ragdollInstant = true;
		if (tracked && IsBurning(a_actor)) {
			std::scoped_lock lock{ g_mutex };
			g_burningDeaths.push_back(a_actor->GetFormID());
		}
		if (tracked) {
			// Its own movement before death (walking, running) is kept; only speed beyond it counts as knockback.
			RE::NiPoint3 velocity{};
			a_actor->GetLinearVelocity(velocity);
			constexpr float kPhysicsPerGameUnit = 0.0142875f;
			std::scoped_lock lock{ g_mutex };
			g_deathMoveVel[a_actor->GetFormID()] = { velocity.x * kPhysicsPerGameUnit, velocity.y * kPhysicsPerGameUnit,
				velocity.z * kPhysicsPerGameUnit };
		}
		(*g_killHook)(a_actor, a_attacker, a_damage, a_sendEvent, a_ragdollInstant);
		std::scoped_lock lock{ g_mutex };
		if (tracked && a_actor->IsDead(false)) {
			g_pendingDeaths.push_back(a_actor->GetFormID());
		} else if (tracked) {
			g_deathMoveVel.erase(a_actor->GetFormID());  // did not die after all (e.g. essential): no leftover entry
			std::erase(g_burningDeaths, a_actor->GetFormID());
		}
	}


	void StartPendingDeaths(float a_now)
	{
		std::vector<RE::TESFormID> pending;
		{
			std::scoped_lock lock{ g_mutex };
			pending.swap(g_pendingDeaths);
		}
		for (const auto formID : pending) {
			const auto actor = RE::TESForm::GetFormByID<RE::Actor>(formID);
			if (!actor || !actor->Get3D() || !GetWorld(actor)) continue;
			static std::mt19937 rng{ std::random_device{}() };
			std::uniform_real_distribution<float> unit{ 0.0f, 1.0f };
			const auto pick = [&](const Range& a_range) { return a_range.min + (a_range.max - a_range.min) * unit(rng); };
			Death death{};
			death.formID = formID;
			death.stumble = unit(rng) < kDeathStumbleChance;
			death.grip = unit(rng) < kDeathGripChance;
			const bool moves = unit(rng) < kTwitchChance;
			// Burned to death (on fire at KillImpl): its own, stronger writhing, whatever the twitch chance rolled.
			bool burned = false;
			{
				std::scoped_lock lock{ g_mutex };
				if (const auto it = std::find(g_burningDeaths.begin(), g_burningDeaths.end(), formID); it != g_burningDeaths.end()) {
					g_burningDeaths.erase(it);
					burned = kBurnDeathTwitchMax > 0.0f || kBurnDeathRollMax > 0.0f;
				}
			}
			if (!death.stumble && !death.grip && !moves && !burned) {
				REX::INFO("DynamicRagdollSystem: {:08X} death: no stumble, grip or twitch rolled; vanilla ragdoll"sv, formID);
				continue;
			}
			death.support = pick(kDeathSupport);
			death.upright = pick(kDeathUpright);
			death.braking = pick(kDeathBraking);
			death.dyingTime = (death.grip || moves || burned) ? pick(kDyingTime) : 0.0f;
			death.twitch = moves ? pick(kTwitchStrength) : 0.0f;
			death.roll = moves ? pick(kRollStrength) : 0.0f;
			if (burned) {
				death.burnedToDeath = true;
				death.twitch = kBurnDeathTwitchMin + (kBurnDeathTwitchMax - kBurnDeathTwitchMin) * unit(rng);
				death.roll = kBurnDeathRollMin + (kBurnDeathRollMax - kBurnDeathRollMin) * unit(rng);
				death.dyingTime = kBurnDeathTimeMin + (kBurnDeathTimeMax - kBurnDeathTimeMin) * unit(rng);
				REX::INFO("DynamicRagdollSystem: {:08X} burned to death: twitch {:.2f} roll {:.2f} for {:.1f}s"sv, formID, death.twitch,
					death.roll, death.dyingTime);
			}
			death.flail = pick(kFlailStrength);
			death.start = a_now;
			// Without a stumble, StepDeath still runs until the bodies are scanned (it then ends the stumble).
			death.end = a_now + (death.stumble ? pick(kDeathStumbleTime) : 5.0f);
			REX::INFO("DynamicRagdollSystem: {:08X} death rolled: stumble {} ({:.2f}s, support {:.2f}, upright {:.2f}, braking {:.2f}) grip {} dying {:.2f}s twitch {:.2f} roll {:.2f}"sv,
				formID, death.stumble, death.end - a_now, death.support, death.upright, death.braking, death.grip, death.dyingTime,
				death.twitch, death.roll);
			std::scoped_lock lock{ g_mutex };
			if (const auto it = g_lastHit.find(formID); it != g_lastHit.end() && a_now - it->second.time < kLastHitWindow) {
				death.hitDir = it->second.dir;
				death.hitBody = it->second.hitBody;
				death.weapon = it->second.weapon;
				death.damage = it->second.damage;
				// Explosion deaths are flung: no balance help (support/upright/braking), the blast push, and the limbs
				// flail while airborne. The stumble machinery still runs for the joints, grip and flailing.
				if (it->second.explosion && unit(rng) >= kExplosionStumbleChance) {
					if (!death.stumble) death.end = a_now + pick(kDeathStumbleTime);
					death.stumble = true;
					death.support = 0.0f;
					death.upright = 0.0f;
					death.braking = 0.0f;
					death.explosionPush = pick(kExplosionPush);
					death.flung = true;
					REX::INFO("DynamicRagdollSystem: {:08X} explosion death: flung at {:.1f} (lift {:.2f})"sv, formID, death.explosionPush, kExplosionLift);
				} else if (kDeathFlingDamage > 0.0f && it->second.damage >= kDeathFlingDamage) {
					// Heavy hits (e.g. a Gauss rifle) blow the body away: same as an explosion but with the weapon's own
					// knockback kept in full instead of a blast push.
					if (!death.stumble) death.end = a_now + pick(kDeathStumbleTime);
					death.stumble = true;
					death.support = 0.0f;
					death.upright = 0.0f;
					death.braking = 0.0f;
					death.flung = true;
					REX::INFO("DynamicRagdollSystem: {:08X} heavy hit ({:.0f} damage, '{}'): flung"sv, formID, it->second.damage, it->second.weapon);
				}
			} else {
				death.hitApplied = true;  // no recent hit (script kill, fall, …): no push
			}
			g_lastHit.erase(formID);
			if (const auto it = g_deathMoveVel.find(formID); it != g_deathMoveVel.end()) {
				death.moveVel = it->second;
				g_deathMoveVel.erase(it);
			}
			ReplaceDeath(std::move(death));
			REX::INFO("DynamicRagdollSystem: {:08X} death stumble started"sv, formID);
		}
	}
}
