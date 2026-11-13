#include "Common.h"
//
// dies -> Hook_KillImpl -> remember pre-death velocity -> not in bleedout anim (IsReallyDead) -> add to death queue (g_pendingDeaths)
// -> world update -> create controller for dead body (StartPendingDeaths) -> manipulate body for target anim
//
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
					// A hand above the wound: the upper arm only partly carried, so the elbow drops. Fully carried, a raised
					// arm (aiming at death) kept its elbow at shoulder height and the hand stuck under the chin. Below the
					// wound (coming up from behind the back to the head) the whole arm is carried, or it can't lift.
					const DeathSample* parent = find(s.ref->parentId.m_value);
					const bool dropElbow = parent && !parent->ref->arm && hand->pos.z > wound->pos.z;
					const auto lift = Scale(Vec3{ g.x, g.y, g.z }, -m * dt * a_strength * (dropElbow ? kGripUpperArmCarry : 1.0f));
					RE::hkVector4f liftImpulse{ lift.x, lift.y, lift.z, 0.0f };
					g_linearImpulse(npWorld, s.ref->id.m_value, &liftImpulse);
					woundImpulse = Sub(woundImpulse, lift);
				}
				// A hand behind the trunk pulled straight at the wound drags the arm into the back and sticks there:
				// first out to its own side past the hips, then round in front of the wound, until it is past the chest.
				Vec3 target = wound->pos;
				const DeathSample* shoulder[2]{};  // left, right upper arm
				for (const auto& s : samples)
					if (const DeathSample* parent = s.ref->arm && s.ref->side ? find(s.ref->parentId.m_value) : nullptr; parent && !parent->ref->arm)
						shoulder[s.ref->side == 'R'] = &s;
				if (const auto [low, high] = TrunkEnds(samples); low && high && low != high && shoulder[0] && shoulder[1]) {
					const Vec3  up = Unit(Sub(high->pos, low->pos)), right = Unit(Sub(shoulder[1]->pos, shoulder[0]->pos));
					const Vec3  forward = Unit(Cross(up, right)), out = Scale(right, hand->ref->side == 'R' ? 1.0f : -1.0f);
					const float behind = -Dot(Sub(hand->pos, high->pos), forward);
					const float detour = std::clamp((behind - kGripLag) / kGripClear, 0.0f, 1.0f);
					const float lateral = Dot(Sub(hand->pos, low->pos), out);
					const float aside = std::clamp((lateral - kGripOut + kGripClear) / kGripClear, 0.0f, 1.0f);  // forward only once past the hips
					const Vec3  sideways = Add(hand->pos, Scale(out, kGripOut));  // a full kGripOut away: nearer pulls too weakly to free it
					const Vec3  around = Add(wound->pos, Add(Scale(forward, kGripAround), Scale(out, kGripOut)));
					target = Add(target, Scale(Sub(Add(sideways, Scale(Sub(around, sideways), aside)), wound->pos), detour));
				}
				const auto relVel = Sub(hand->motion.linear, wound->motion.linear);
				Vec3 deltaV = Scale(Sub(Scale(Sub(target, hand->pos), kGripKp), Scale(relVel, kGripKd)), dt * a_strength);
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


	// World geometry ray between two points (physics units), via the engine's pick ray (bhkPickData +
	// TESObjectCELL::Pick; start/end in game units, the pick scales them). a_fraction: 0 at a_from, 1 at a_to.
	// Takes the world's read lock: never call with the world write lock held.
	bool PickFraction(RE::TESObjectCELL* a_cell, Vec3 a_from, Vec3 a_to, float& a_fraction)
	{
		constexpr float kGame = 69.99125f;  // game units per physics unit
		RE::bhkPickData pick{};
		pick.SetStartEnd(RE::NiPoint3{ a_from.x * kGame, a_from.y * kGame, a_from.z * kGame },
			RE::NiPoint3{ a_to.x * kGame, a_to.y * kGame, a_to.z * kGame });
		pick.castQuery.m_filterData.m_collisionFilterInfo = static_cast<std::uint32_t>(RE::COL_LAYER::kPathingPick);
		// Explicit layer list (bhkWorld::Pick then uses hknpBSCustomCollisionFilter: one bit per layer; bodies with a
		// collision group, like ragdoll parts, are only hit if their layer is listed; ungrouped world geometry always).
		using L = RE::COL_LAYER;
		constexpr auto bit = [](L a_layer) { return std::uint64_t{ 1 } << static_cast<std::uint32_t>(a_layer); };
		pick.customCollideLayers = bit(L::kStatic) | bit(L::kAnimStatic) | bit(L::kTrees) | bit(L::kProps) | bit(L::kTerrain) |
		                           bit(L::kGround) | bit(L::kStairHelper);
		static_cast<void>(a_cell->Pick(pick));  // the hit is read from pick, not the returned object
		if (!pick.HasHit()) return false;
		a_fraction = pick.GetHitFraction();
		return std::isfinite(a_fraction);
	}


	// Ground height under a point (physics units). Returns false when nothing is hit within kRayDown.
	bool CastGround(RE::TESObjectCELL* a_cell, Vec3 a_pos, float& a_ground)
	{
		float fraction = 1.0f;
		if (!PickFraction(a_cell, Vec3{ a_pos.x, a_pos.y, a_pos.z + kRayUp }, Vec3{ a_pos.x, a_pos.y, a_pos.z - kRayDown }, fraction))
			return false;
		a_ground = a_pos.z + kRayUp - (kRayUp + kRayDown) * fraction;
		return true;
	}


	// The body's front direction (trunk up × left → right thigh line; standing: up z × right x = front y) and the
	// stomach (middle of the trunk). False without both thighs or a trunk.
	bool CrawlFront(const std::vector<DeathSample>& a_samples, Vec3& a_front, Vec3& a_stomach)
	{
		const auto [low, high] = TrunkEnds(a_samples);
		const DeathSample* thigh[2]{};
		for (const auto& s : a_samples)
			if (s.ref->leg && !s.ref->knee && !s.ref->foot && s.ref->side) thigh[s.ref->side == 'L' ? 0 : 1] = &s;
		if (!low || !high || low == high || !thigh[0] || !thigh[1]) return false;
		const auto up = Unit(Sub(high->pos, low->pos));
		auto right = Sub(thigh[1]->pos, thigh[0]->pos);
		right = Sub(right, Scale(up, Dot(right, up)));
		if (Length(right) < 0.05f) return false;
		a_front = Cross(up, Unit(right));
		a_stomach = Scale(Add(low->pos, high->pos), 0.5f);
		return true;
	}


	// Face-down ray for a pending crawl: from the stomach out of the body's front. Cast outside the world lock.
	void CastCrawlRay(Death& a_death, RE::Actor* a_actor, RE::bhkWorld* a_world)
	{
		a_death.crawlRayValid = false;
		auto* npWorld = a_world ? a_world->m_worldNP.get() : nullptr;
		auto* cell = a_actor->GetParentCell();
		if (!npWorld || !cell) return;
		auto& worldBodies = npWorld->m_bodyManager.m_bodies;
		std::vector<DeathSample> samples;
		for (const auto& body : a_death.bodies) {
			if (!FindBody(worldBodies, body)) continue;
			if (a_death.Detached(body.id.m_value)) continue;
			samples.push_back({ &body, MotionState{}, BodyPosition(worldBodies[body.index]) });
		}
		Vec3 front{}, stomach{};
		if (!CrawlFront(samples, front, stomach)) return;
		float fraction = 1.0f;
		const bool hit = PickFraction(cell, stomach, Add(stomach, Scale(front, kCrawlRayLength)), fraction);
		a_death.crawlFront = front;
		a_death.crawlRayDist = hit ? fraction * kCrawlRayLength : 1.0e9f;
		a_death.crawlRayValid = true;
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
			if (!FindBody(worldBodies, body)) continue;
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
		const float wanted = std::clamp((a_lean - kBraceLean) / (kBraceFull - kBraceLean), 0.0f, 1.0f) * kDeathBrace * a_scale;
		// Eased in and out: switching on at full strength late in the stumble yanked the arms and, through the
		// reaction on the chest, swung the torso sideways.
		const float ramp = kDeathBrace * dt / kBraceRampTime;
		a_death.braceLevel += std::clamp(wanted - a_death.braceLevel, -ramp, ramp);
		const float amount = a_death.braceLevel;
		if (amount <= 1.0e-4f) return;
		const auto [low, high] = TrunkEnds(samples);  // pelvis / SPINE1, chest
		if (!low || !high || low == high) return;
		// Fall direction = where the trunk leans; nearly upright, where the chest is moving.
		Vec3 fall{ high->pos.x - low->pos.x, high->pos.y - low->pos.y, 0.0f };
		if (Length(fall) < 0.05f) fall = Vec3{ high->motion.linear.x, high->motion.linear.y, 0.0f };
		fall = Length(fall) < 1.0e-3f ? a_death.braceFall : Unit(fall);
		// Turned slowly: following the (noisy) lean/chest velocity each frame swung the guard points side to side,
		// and the chest with them; a feedback loop with the reaction impulse.
		if (Length(a_death.braceFall) > 0.5f) {
			const auto eased = Add(a_death.braceFall, Scale(Sub(fall, a_death.braceFall), std::min(1.0f, kBraceTurnRate * dt)));
			fall = Length(eased) > 0.1f ? Unit(eased) : a_death.braceFall;
		}
		if (Length(fall) < 0.5f) return;
		a_death.braceFall = fall;
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
			// Feet: height of the lower foot (dying mid-stride one is in the air: the average put the ground ~5 cm up, so
			// that foot never counted as lifted); each foot's flat place relative to the feet's centre, capped so a stride
			// isn't kept; and the foot mid-swing at death (clearly faster than the other), which takes the first step.
			float footSpeed[2]{};
			std::uint32_t footId[2]{};
			int footCount = 0;
			for (const auto& body : a_stumble.bodies) {
				if (!body.foot || !valid(body) || footCount >= 2) continue;
				const auto& b = worldBodies[body.index];
				const Vec3 pos = BodyPosition(b);
				a_stumble.feetCaptured.z = std::min(a_stumble.feetCaptured.z, pos.z);
				Vec3 offset{ pos.x - a_stumble.feetCaptured.x, pos.y - a_stumble.feetCaptured.y, 0.0f };
				if (const float n = Length(offset); n > kStanceMax) offset = Scale(offset, kStanceMax / n);
				a_stumble.stanceOffsets.push_back({ body.id.m_value, offset });
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
		if (a_stumble.walk && kDeathWalkEndDamage > 0.0f && a_stumble.walkDamage >= kDeathWalkEndDamage) {
			// Shot enough while walking: the support drops at once; the dying phase follows as after a fall.
			REX::INFO("DynamicRagdollSystem: {:08X} death walk: {:.0f} damage taken; collapses"sv, a_stumble.formID, a_stumble.walkDamage);
			EndDeathWalk(a_stumble.formID);
			a_stumble.walk = false;
			a_stumble.end = a_now;
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
				a_stumble.walk = false;
				a_stumble.stumble = true;  // the stumble it would have had instead
				a_stumble.end = a_now + Random(kDeathStumbleTime.min, kDeathStumbleTime.max);
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
		const float fade = std::clamp((a_stumble.end - a_now) / std::max(a_stumble.end - a_stumble.start, 0.01f), 0.0f, 1.0f);
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
			// Horizontal: braked toward a stagger along the stagger line instead of to a standstill, which stopped the
			// body within a step and left the feet shuffling on the spot. It fades with strength.
			const Vec3 staggerDir = Length(a_stumble.staggerLine) > 0.5f ? a_stumble.staggerLine : Unit(Vec3{ a_stumble.hitDir.x, a_stumble.hitDir.y, 0.0f });
			const Vec3 stagger = Scale(staggerDir, kStaggerSpeed * fade);
			const Vec3 dCom{ -a_stumble.braking * (comVel.x - stagger.x) * dt * catchable, -a_stumble.braking * (comVel.y - stagger.y) * dt * catchable,
				grounded ? support : std::min(0.0f, -kHeightKd * comVel.z * dt) };  // airborne: brake the launch only
			// The stagger line: the way the body moves just after the hit (its push plus whatever it was doing, e.g.
			// walking), fixed for the whole stumble. Steps aimed wherever the pelvis drifted walked it round in circles.
			if (Length(a_stumble.staggerLine) < 0.5f && a_stumble.hitApplied) {
				a_stumble.staggerLine = Unit(Vec3{ comVel.x, comVel.y, 0.0f });
				if (Length(a_stumble.staggerLine) < 0.5f) a_stumble.staggerLine = Unit(Vec3{ a_stumble.hitDir.x, a_stumble.hitDir.y, 0.0f });
				if (Length(a_stumble.staggerLine) < 0.5f) a_stumble.staggerLine = Vec3{ 0.0f, 1.0f, 0.0f };
				a_stumble.staggerFrom = Vec3{ feet.x, feet.y, 0.0f };
			}
			const auto flat = [](Vec3 a_v) { return Vec3{ a_v.x, a_v.y, 0.0f }; };
			const auto findSample = [&](std::uint32_t a_id) -> const Sample* {
				for (const auto& s : samples)
					if (s.ref->id.m_value == a_id) return &s;
				return nullptr;
			};
			// Leg bodies above a foot: calf (knee) and thigh (hip). Body origins sit on the joints.
			const auto legOf = [&](const Sample& a_foot, const Sample*& a_calf, const Sample*& a_thigh) {
				a_calf = findSample(a_foot.ref->parentId.m_value);
				a_thigh = a_calf ? findSample(a_calf->ref->parentId.m_value) : nullptr;
				return a_calf && a_thigh;
			};
			Vec3 hips{};  // flat, left to right across the thighs
			for (const auto& s : samples)
				if (s.ref->leg && !s.ref->knee && !s.ref->foot && s.ref->side) hips = Add(hips, Scale(s.pos, s.ref->side == 'R' ? 1.0f : -1.0f));
			hips = Unit(flat(hips));
			const auto ownSide = [&](const BodyRef& a_ref) { return Scale(hips, a_ref.side == 'R' ? 1.0f : -1.0f); };
			// Stepping: when the capture point (where the COM will come to rest) passes the feet along the stagger line,
			// swing a foot to it. A foot that must step (see `up`) steps at once.
			if (grounded && !a_stumble.stepFoot && fade > 0.3f && catchable > 0.3f && Length(a_stumble.staggerLine) > 0.5f) {
				const Sample* footSamples[2]{};
				int footCount = 0;
				for (const auto& s : samples)
					if (s.ref->foot && footCount < 2) footSamples[footCount++] = &s;
				if (footCount == 2) {
					const Vec3& line = a_stumble.staggerLine;
					const float look = std::sqrt(std::max(com.z - feet.z, 0.3f) / std::max(-g.z, 1.0f));
					// Carried by the pelvis's velocity, not the COM's: a shot to the upper body bends it at the hips first,
					// and stepping for that sent a foot out where the hips never followed (a stiff, stretched leg that then
					// shuffled back). Only the motion along the line counts; sideways drift is braked.
					const float at = Dot(Sub(flat(com), a_stumble.staggerFrom), line) + Dot(trunkMotion.linear, line) * look;
					const Vec3 capture = Add(a_stumble.staggerFrom, Scale(line, at));
					const float a0 = Dot(Sub(flat(footSamples[0]->pos), a_stumble.staggerFrom), line);
					const float a1 = Dot(Sub(flat(footSamples[1]->pos), a_stumble.staggerFrom), line);
					const float beyond = std::max(at - std::max(a0, a1), std::min(a0, a1) - at);  // > 0: past the feet
					// Must step next: up in the air; dying mid-stride, out of its hip's reach (it can't hold the body up:
					// the body sank onto it in a split and pinned the leg straight); or, for the first step, the foot
					// mid-swing at death (only a few cm up: it flew on forward as a straight kick while the other stepped).
					const auto up = [&](const Sample* a_s) {
						const Sample* calf = nullptr;
						const Sample* thigh = nullptr;
						const bool stranded = a_stumble.swingAtDeath && legOf(*a_s, calf, thigh) && Length(flat(Sub(a_s->pos, thigh->pos))) > kStepReach;
						return a_s->pos.z - footLevel(a_s->ref->id.m_value) > kLiftedFoot || stranded ||
						       (a_stumble.steps == 0 && a_s->ref->id.m_value == a_stumble.swingAtDeath);
					};
					const bool lifted[2]{ up(footSamples[0]), up(footSamples[1]) };
					if (lifted[0] != lifted[1] || (a_now >= a_stumble.nextStep && beyond > kStepTrigger)) {
						// Each foot keeps its standing place relative to the line and lands kStepOvershoot past the capture point.
						const Vec3 dir = Scale(line, at >= 0.5f * (a0 + a1) ? 1.0f : -1.0f);
						const float maxStep = std::min(kStepMaxLength * (1.0f + 0.25f * speed), 2.0f * kStepMaxLength);  // faster: longer strides
						const auto land = [&](int a_i) {
							const Sample* s = footSamples[a_i];
							Vec3 offset{};
							for (const auto& [id, o] : a_stumble.stanceOffsets)
								if (id == s->ref->id.m_value) offset = o;
							Vec3 step = flat(Sub(Add(Add(capture, Scale(dir, kStepOvershoot)), offset), s->pos));
							if (const float n = Length(step); n > maxStep) step = Scale(step, maxStep / n);
							Vec3 to = Add(s->pos, step);
							// Never out of the leg's reach: a spot the foot can't get to from the hip (a leg the hit kicked far
							// out) left the leg pointing at it, straight and hanging in the air.
							const Sample* calf = nullptr;
							const Sample* thigh = nullptr;
							if (legOf(*s, calf, thigh)) {
								const Vec3 hip = flat(thigh->pos), out = Sub(flat(to), hip);
								if (const float n = Length(out); n > kStepReach) to = Add(hip, Scale(out, kStepReach / n));
							}
							// Stay on its own side of the other foot, across the hips as they are now, or it tangles in that leg.
							const Vec3 own = ownSide(*s->ref);
							if (const float gap = Dot(flat(Sub(to, footSamples[1 - a_i]->pos)), own); gap < kFootGap) to = Add(to, Scale(own, kFootGap - gap));
							to.z = footLevel(s->ref->id.m_value);  // a lifted foot comes down
							return to;
						};
						// The foot that can actually move farther steps (a lifted one always). Judged after the limits above:
						// on a sideways push the trailing foot is farthest from its spot but blocked by the other foot, and
						// choosing it sent it again and again behind the other leg. Now the leading foot goes out first.
						const auto need = [&](int a_i) { return Length(flat(Sub(land(a_i), footSamples[a_i]->pos))) + (lifted[a_i] ? 9.0f : 0.0f); };
						const int pick = need(0) > need(1) ? 0 : 1;
						const Sample* swing = footSamples[pick];
						a_stumble.stepFoot = swing->ref->id.m_value;
						a_stumble.stepSide = swing->ref->side;
						a_stumble.stepFrom = swing->pos;
						a_stumble.stepTo = land(pick);
						a_stumble.stepStart = a_now;
						a_stumble.stepGroundTried = false;  // landing ray next frame (UpdateGround)
						a_stumble.stepGroundHit = false;
						a_stumble.stepTime = kStepTime / (1.0f + 0.3f * speed);  // quicker steps when moving fast
						REX::INFO("DynamicRagdollSystem: {:08X} death step {} with {} foot, length {:.2f}{}"sv, a_stumble.formID, ++a_stumble.steps,
							swing->ref->side ? swing->ref->side : '?', Length(flat(Sub(a_stumble.stepTo, swing->pos))), lifted[pick] ? " (foot up)" : "");
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
					// stepped onto. No hit, or a change too big to be a step: the foot's own ground level.
					float toZ = to.z;
					if (rays && a_stumble.stepGroundHit) {
						const float level = rest + a_stumble.stepGround - a_stumble.groundAtDeath;
						if (std::abs(level - to.z) < kStepMaxRise) toZ = level;
					}
					// The foot swings in under the hip on its way (a curve through the hip's current spot), like a real step.
					// A straight line from a trailing foot lifted it while the leg was still stretched out behind: a stiff leg.
					const Sample* calf = nullptr;
					const Sample* thigh = nullptr;
					const bool leg = legOf(*swing, calf, thigh);
					const Vec3 under = leg ? Vec3{ thigh->pos.x, thigh->pos.y, from.z } : Scale(Add(from, to), 0.5f);
					const float u0 = (1.0f - phase) * (1.0f - phase), u1 = 2.0f * phase * (1.0f - phase), u2 = phase * phase;
					Vec3 desiredPos{ from.x * u0 + under.x * u1 + to.x * u2, from.y * u0 + under.y * u1 + to.y * u2,
						from.z + (toZ - from.z) * phase + kStepHeight * std::sin(3.14159265f * phase) };
					// The way there stays on its own side of the planted foot too (swinging behind it tangled the legs).
					for (const auto& s : samples)
						if (s.ref->foot && s.ref->id.m_value != a_stumble.stepFoot) {
							const Vec3 own = ownSide(*swing->ref);
							if (const float gap = Dot(flat(Sub(desiredPos, s.pos)), own); gap < kFootGap) desiredPos = Add(desiredPos, Scale(own, kFootGap - gap));
						}
					const Vec3 pathV{ (2.0f * (1.0f - phase) * (under.x - from.x) + 2.0f * phase * (to.x - under.x)) / a_stumble.stepTime,
						(2.0f * (1.0f - phase) * (under.y - from.y) + 2.0f * phase * (to.y - under.y)) / a_stumble.stepTime, (toZ - from.z) / a_stumble.stepTime };
					// The knee goes where two-bone IK puts it for that ankle point, bent toward the body's front. Driving the
					// foot alone lifted it by swinging the whole straight leg from the hip (a stiff goose step).
					std::vector<std::pair<const Sample*, Vec3>> drive{ { &*swing, desiredPos } };
					const auto* thighTarget = leg ? findTarget(thigh->ref->id.m_value) : nullptr;
					const auto* calfTarget = leg ? findTarget(calf->ref->id.m_value) : nullptr;
					const auto* footTarget = findTarget(swing->ref->id.m_value);
					if (thighTarget && calfTarget && footTarget) {
						const float upper = Length(Sub(calfTarget->pos, thighTarget->pos)), lower = Length(Sub(footTarget->pos, calfTarget->pos));
						Vec3 reach = Sub(desiredPos, thigh->pos);
						if (const float n = Length(reach); n > 0.999f * (upper + lower)) reach = Scale(reach, 0.999f * (upper + lower) / n);
						const float d = std::max(Length(reach), 1.0e-3f);
						const Vec3 u = Scale(reach, 1.0f / d), front = Cross(Vec3{ 0.0f, 0.0f, 1.0f }, hips);
						Vec3 bendTo = Unit(Sub(front, Scale(u, Dot(front, u))));
						if (Length(bendTo) < 0.5f) bendTo = front;
						const float along = (upper * upper - lower * lower + d * d) / (2.0f * d);
						drive.push_back({ calf, Add(thigh->pos, Add(Scale(u, along), Scale(bendTo, std::sqrt(std::max(0.0f, upper * upper - along * along))))) });
					}
					Vec3 legImpulse{};
					for (const auto& [s, target] : drive) {
						Vec3 deltaV = Sub(Add(pathV, Scale(Sub(target, s->pos), kSwingKp)), s->motion.linear);
						const float n = Length(deltaV), cap = kSwingMaxAccel * dt;
						if (!std::isfinite(n) || n < 1.0e-4f) continue;
						if (n > cap) deltaV = Scale(deltaV, cap / n);
						const auto impulse = Scale(deltaV, 1.0f / s->motion.inverseMass);
						RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
						g_linearImpulse(npWorld, s->ref->id.m_value, &nativeImpulse);
						legImpulse = Add(legImpulse, impulse);
					}
					// Hip-muscle reaction on the trunk: an unreacted swing push added momentum toward the capture
					// point every step, so the body sped up (longer steps, more push) and slid past its planted leg.
					float trunkMass = 0.0f;
					for (const auto& s : samples)
						if (s.ref->pelvis || s.ref->spine) trunkMass += 1.0f / s.motion.inverseMass;
					if (trunkMass > 0.0f) {
						const auto backV = Scale(legImpulse, -1.0f / trunkMass);  // same velocity change on each trunk body
						for (const auto& s : samples) {
							if (!s.ref->pelvis && !s.ref->spine) continue;
							const auto back = Scale(backV, 1.0f / s.motion.inverseMass);
							RE::hkVector4f nativeBack{ back.x, back.y, back.z, 0.0f };
							g_linearImpulse(npWorld, s.ref->id.m_value, &nativeBack);
						}
					}
				}
			}

			// All help acts through the planted feet: the opposite impulse goes into the stance feet, where the ground
			// takes it (like a leg pushing). No planted foot = no help, and sideways help is limited by foot friction,
			// so the body cannot hover or be held up against its momentum by an invisible hand. Only a foot really down
			// takes it: one already lifted carries no weight, so the push slid it out and straightened the leg.
			std::vector<const Sample*> stance;
			for (const auto& s : samples)
				if (s.ref->foot && s.ref->id.m_value != a_stumble.stepFoot && s.pos.z - footLevel(s.ref->id.m_value) < kLiftedFoot &&
				    footLevel(s.ref->id.m_value) - s.pos.z < kGroundTolerance)
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
			// A knee whose foot is off the ground (and not stepping) carries no weight: it folds loose. Held at the death
			// pose it stayed locked straight and stuck out while the body rolled off it.
			bool hanging = false;
			if (joint.knee && !swingLeg)
				for (const auto& s : samples)
					if (s.ref->foot && s.ref->parentId.m_value == joint.id.m_value) hanging = s.pos.z - footLevel(s.ref->id.m_value) > kLiftedFoot;
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
					if (hanging) bend = std::max(bend, kHangingKneeBend);
					targetRelative = Multiply(AxisAngle(axis, bend * 0.0174533f), targetRelative);
				}
			}
			const auto errorWorld = Rotate(parentRotation, RotationError(currentRelative, targetRelative));
			const auto relativeOmega = Sub(Rotate(childMotion.orientation, childMotion.angular),
				Rotate(parentMotion.orientation, parentMotion.angular));
			// Knees get a firmer, less faded drive: at low stiffness a loaded leg never reached its bend. A stepping or
			// hanging knee much firmer still, or the foot being pulled along straightened it.
			const float stiffness = joint.knee ? std::max(kDeathStiffness, kKneeMinStiffness) * (swingLeg || hanging ? kSwingKneeStiffness : 1.0f) :
			                                     kDeathStiffness;
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




	// Whole-body roll about the trunk's long axis (pelvis → chest, through the trunk centre) at a_accel rad/s^2
	// (sign = direction). Every body gets the velocity of the rig rotating about the axis; torquing only the trunk
	// could not turn the limbs hanging off it. False (nothing applied) unless the trunk lies flat.
	bool RollTrunk(RE::hknpBSWorld* npWorld, const std::vector<DeathSample>& samples, float dt, float a_accel, bool a_armsRoll = false)
	{
		const auto [low, high] = TrunkEnds(samples);  // pelvis / SPINE1, chest
		if (!low || !high || low == high) return false;
		const auto axis = Unit(Sub(high->pos, low->pos));
		if (std::abs(axis.z) > 0.3f) return false;  // not lying flat: rolling about this axis would spin or flip it
		const auto centre = Scale(Add(high->pos, low->pos), 0.5f);
		const auto dOmega = Scale(axis, a_accel * dt);
		const float cap = kRollMaxAccel * dt;
		for (const auto& sample : samples) {
			auto r = Sub(sample.pos, centre);
			r = Sub(r, Scale(axis, Dot(r, axis)));  // perpendicular offset from the axis
			auto deltaV = Cross(dOmega, r);
			if (const float n = Length(deltaV); n > cap) deltaV = Scale(deltaV, cap / n);
			// Legs are only dragged along by the hips: no lift and no spin of their own. Limp legs given the
			// full rigid-body roll kept flying up after the burst. Arms too: an outstretched hand (far from the axis,
			// light) was flung first and dragged the body over by the shoulder. a_armsRoll (bBurnRollArms / bDeathRollArms) keeps the old arm drive.
			const bool limb = sample.ref->leg || (sample.ref->arm && !a_armsRoll);
			if (limb) deltaV.z = std::min(deltaV.z, 0.0f);
			const auto impulse = Scale(deltaV, 1.0f / sample.motion.inverseMass);
			RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
			g_linearImpulse(npWorld, sample.ref->id.m_value, &nativeImpulse);
			// Each body also turns about the axis itself: the trunk bodies sit on the axis, so the linear
			// part alone moved the limbs while the torso never turned over.
			if (g_angularImpulse && !limb) {
				float inverse[3][3]{};
				AddWorldInverse(sample.motion, inverse);
				Vec3 angular{};
				if (Solve3(inverse, dOmega, angular)) {
					RE::hkVector4f angularImpulse{ angular.x, angular.y, angular.z, 0.0f };
					g_angularImpulse(npWorld, sample.ref->id.m_value, &angularImpulse);
				}
			}
		}
		return true;
	}


	// Pending crawl: decide from the face-down ray (CastCrawlRay). Hits the ground close = face down: crawl. Front
	// toward the sky = face up: no crawl. Otherwise it lies on its side: roll it toward face down and look again
	// once it settles, until kCrawlTurnTime. Returns true once decided.
	bool CheckCrawl(Death& a_death, RE::hknpBSWorld* npWorld, const std::vector<DeathSample>& samples, float dt, float a_now)
	{
		const auto finish = [&](bool a_crawl, std::string_view a_why) {
			a_death.crawlDecided = true;
			a_death.crawling = a_crawl;
			REX::INFO("DynamicRagdollSystem: {:08X} crawl: {} (ray {}, front z {:.2f}, {} turn rolls)"sv, a_death.formID, a_why,
				a_death.crawlRayDist < 1.0e8f ? std::format("{:.2f}", a_death.crawlRayDist) : "no hit"s, a_death.crawlFront.z, a_death.crawlTurns);
			if (a_crawl) {
				a_death.grip = false;  // the arms crawl
				a_death.crawlStart = std::max(a_now, a_death.downTime + kCrawlDelay);
				a_death.crawlTime = Random(kCrawlTime.min, kCrawlTime.max);
				a_death.dyingTime = a_death.crawlStart + a_death.crawlTime - a_death.downTime;  // the entry ends with the crawl
				REX::INFO("DynamicRagdollSystem: {:08X} crawl starts in {:.1f}s, lasts {:.1f}s"sv, a_death.formID, a_death.crawlStart - a_now, a_death.crawlTime);
			}
			else if (!a_death.grip && a_death.twitch <= 0.0f && a_death.roll <= 0.0f) a_death.dyingTime = 0.0f;  // nothing to play
			return true;
		};
		if (a_death.headOffBefore) return finish(false, "headless, no crawl");  // before any ray or turn-over
		if (a_death.headShotKill) return finish(false, "killed by a head shot, no crawl");
		if (a_now < a_death.rollEnd) {  // turning over
			RollTrunk(npWorld, samples, dt, a_death.rollSign * kRollAccel);
			return false;
		}
		if (!a_death.crawlRayValid || a_now < a_death.rollEnd + kCrawlSettle) {
			return a_now >= a_death.downTime + kCrawlTurnTime ? finish(false, "no face-down check in time") : false;
		}
		if (a_death.crawlRayDist < kCrawlDownDist) return finish(true, "face down, crawling");
		if (a_death.crawlFront.z > 0.5f) return finish(false, "face up, no crawl");
		const auto [low, high] = TrunkEnds(samples);
		if (!low || !high || low == high || std::abs(Unit(Sub(high->pos, low->pos)).z) > 0.3f) return finish(false, "not lying flat, no crawl");
		if (a_now >= a_death.downTime + kCrawlTurnTime) return finish(false, "still on its side, no crawl");
		// On its side: roll so the front turns down (front changes by ω × front; its z must drop).
		const auto axis = Unit(Sub(high->pos, low->pos));
		a_death.rollSign = Cross(axis, a_death.crawlFront).z > 0.0f ? -1.0f : 1.0f;
		a_death.rollEnd = a_now + kRollBurst;
		++a_death.crawlTurns;
		REX::INFO("DynamicRagdollSystem: {:08X} crawl: on its side (ray {}, front z {:.2f}), rolling face down"sv, a_death.formID,
			a_death.crawlRayDist < 1.0e8f ? std::format("{:.2f}", a_death.crawlRayDist) : "no hit"s, a_death.crawlFront.z);
		return false;
	}


	// Crawl (lying face down, instead of twitches/rolls): the arms take turns reaching ahead of the chest and pulling
	// back to it, the legs drag. Hand drives are paired with the chest like the brace.
	// shortcut: the body moves forward by a nudge on the trunk while an arm pulls, not by the hands gripping the
	// ground; replace with hand/ground friction if it reads as sliding.
	void ApplyCrawl(Death& a_death, RE::hknpBSWorld* npWorld, const std::vector<DeathSample>& samples, float dt, float a_now, float a_strength)
	{
		if (a_strength <= 0.0f) return;
		const auto [low, high] = TrunkEnds(samples);  // pelvis / SPINE1, chest
		if (!low || !high || low == high) return;
		Vec3 forward{ high->pos.x - low->pos.x, high->pos.y - low->pos.y, 0.0f };  // head first
		if (Length(forward) < 0.05f) return;
		forward = Unit(forward);
		const auto right = Cross(Vec3{ 0.0f, 0.0f, -1.0f }, forward);  // the body's right when its front faces down
		const auto& g = npWorld->m_gravity;
		const float cycle = (a_now - a_death.crawlStart) / std::max(kCrawlInterval, 0.1f);
		float pull = 0.0f;
		Vec3 chestImpulse{};
		for (const auto& hand : samples) {
			if (!hand.ref->hand || !hand.ref->side) continue;
			const int k = hand.ref->side == 'L' ? 0 : 1;
			const float phase = cycle + 0.5f * k - std::floor(cycle + 0.5f * k);
			const bool reach = phase < 0.5f;
			float armMass = 0.0f;
			for (const auto& s : samples)
				if (s.ref->arm && s.ref->side == hand.ref->side) armMass += 1.0f / s.motion.inverseMass;
			const DeathSample* forearm = nullptr;  // the hand's parent body: driving it swings the whole arm forward
			for (const auto& s : samples)
				if (s.ref->id.m_value == hand.ref->parentId.m_value && s.ref->arm) forearm = &s;
			const float sideSign = k ? 1.0f : -1.0f;
			auto reachTarget = Add(Add(high->pos, Scale(forward, kCrawlReach)), Scale(right, kCrawlSide * sideSign));
			reachTarget.z = high->pos.z + kCrawlLift;
			// Hand reaching: damped relative to the chest. Planted: damped absolutely, so the hand stays put.
			const auto drive = [&](const DeathSample& a_body, Vec3 a_target, float a_mass, bool a_relative) {
				const auto vel = a_relative ? Sub(a_body.motion.linear, high->motion.linear) : a_body.motion.linear;
				Vec3 deltaV = Scale(Sub(Scale(Sub(a_target, a_body.pos), kBraceKp), Scale(vel, kBraceKd)), dt * a_strength * kCrawlArmStrength);
				const float n = Length(deltaV), cap = kBraceMaxAccel * dt;
				if (!std::isfinite(n) || n < 1.0e-4f) return;
				if (n > cap) deltaV = Scale(deltaV, cap / n);
				const auto impulse = Scale(deltaV, std::max(a_mass, 1.0f / a_body.motion.inverseMass));
				RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
				g_linearImpulse(npWorld, a_body.ref->id.m_value, &nativeImpulse);
				chestImpulse = Sub(chestImpulse, impulse);
			};
			if (reach) {
				a_death.crawlPlanted[k] = false;
				// Stuck at a joint limit (not getting closer): leave the arm limp for the rest of this reach instead of
				// holding it twisted and stiff against the limit.
				const float error = Length(Sub(reachTarget, hand.pos));
				if (a_death.crawlBest[k] <= 0.0f) {  // new reach
					a_death.crawlBest[k] = error;
					a_death.crawlBestTime[k] = a_now;
					a_death.crawlStalled[k] = false;
				} else if (error < a_death.crawlBest[k] - kCrawlStallGain) {
					a_death.crawlBest[k] = error;
					a_death.crawlBestTime[k] = a_now;
				} else if (a_now - a_death.crawlBestTime[k] > kCrawlStall) {
					a_death.crawlStalled[k] = true;
				}
				if (a_death.crawlStalled[k]) continue;
				// Hand or elbow up over the back (a tilted body, or the arm swung behind): not driven or held up this
				// frame, so it drops instead of locking up there.
				if (hand.pos.z - high->pos.z > kCrawlArmMaxHeight || (forearm && forearm->pos.z - high->pos.z > kCrawlArmMaxHeight)) continue;
				// Elbow halfway between the shoulder and the reach target, so the arm stretches out instead of the
				// forearm just lifting.
				const auto shoulder = Add(Add(high->pos, Scale(forward, kCrawlShoulder)), Scale(right, kCrawlSide * sideSign));
				auto elbow = Add(shoulder, Scale(Sub(reachTarget, shoulder), kCrawlElbow));
				elbow.z = high->pos.z + kCrawlLift;
				drive(hand, reachTarget, forearm ? 0.5f * armMass : armMass, true);
				if (forearm) drive(*forearm, elbow, 0.5f * armMass, true);
				// Carry the arm's weight while it reaches, paid by the chest.
				for (const auto& s : samples) {
					if (!s.ref->arm || s.ref->side != hand.ref->side) continue;
					const auto lift = Scale(Vec3{ g.x, g.y, g.z }, -dt * a_strength / s.motion.inverseMass);
					RE::hkVector4f liftImpulse{ lift.x, lift.y, lift.z, 0.0f };
					g_linearImpulse(npWorld, s.ref->id.m_value, &liftImpulse);
					chestImpulse = Sub(chestImpulse, lift);
				}
			} else {
				if (!a_death.crawlPlanted[k]) {
					a_death.crawlPlanted[k] = true;
					a_death.crawlPlant[k] = hand.pos;
					// Only a hand planted ahead of the chest pulls; one left behind (first half-cycle, stuck reach) stays
					// limp, or pinning it held the arm locked behind the body.
					const float ahead = Dot(Sub(hand.pos, high->pos), forward);
					a_death.crawlPulling[k] = ahead >= kCrawlMinPlant;
					if (a_death.crawlPulling[k]) ++a_death.crawlPulls;
					// Tuning: how far the reach got (0 = reached its target) and how fast the body is moving.
					REX::INFO("DynamicRagdollSystem: {:08X} crawl {} hand {:.2f} short of its reach target{} (ahead of chest {:.2f}): {}, trunk speed {:.2f}, leg/head pushes {}/{} accepted"sv,
						a_death.formID, hand.ref->side, Length(Sub(reachTarget, hand.pos)), a_death.crawlStalled[k] ? " (stalled)" : "", ahead,
						a_death.crawlPulling[k] ? std::format("pull {}", a_death.crawlPulls) : "no pull"s, Dot(high->motion.linear, forward),
						a_death.crawlLimbHits, a_death.crawlLimbTries);
					a_death.crawlLimbHits = a_death.crawlLimbTries = 0;
					a_death.crawlBest[k] = 0.0f;  // the next reach starts fresh
				}
				if (a_death.crawlPulling[k]) {
					pull = std::max(pull, std::sin((phase - 0.5f) * 6.2831853f));  // 0 → 1 → 0
					drive(hand, a_death.crawlPlant[k], armMass, false);
				}
			}
		}
		RE::hkVector4f toChest{ chestImpulse.x, chestImpulse.y, chestImpulse.z, 0.0f };
		g_linearImpulse(npWorld, high->ref->id.m_value, &toChest);
		// Ankles bend with each pull (toes tuck under and dig in), relaxing while the arms reach. Dorsiflexion turns the
		// foot about the body's right axis (standing: right x, toes y → up).
		if (pull > 0.0f) {
			for (const auto& joint : a_death.joints) {
				if (!joint.foot || a_death.Detached(joint.id.m_value)) continue;
				BodyRef parentRef{};
				parentRef.index = joint.parentIndex;
				parentRef.id = joint.parentId;
				ApplyJointImpulse(npWorld, joint, parentRef, Scale(right, kCrawlAnkleAccel * pull * a_strength * dt),
					kCrawlLimbMaxTorque * a_strength * dt);
			}
		}
		// Legs and head: each joint is brought to a target speed about one axis (relative to its parent), so the motion
		// follows the cycle without winding up against a limit. Axes in the face-down frame: hip abduction turns the
		// thigh about the vertical (right leg +z), knee flexion about -right, neck lift about +right, turn to the right
		// about -z.
		const auto find = [&](std::uint32_t a_id) -> const DeathSample* {
			for (const auto& s : samples)
				if (s.ref->id.m_value == a_id) return &s;
			return nullptr;
		};
		const auto spin = [&](const BodyRef& a_child, Vec3 a_axis, float a_want) {
			const auto* child = find(a_child.id.m_value);
			const auto* parent = find(a_child.parentId.m_value);
			if (!child || !parent) return;
			const auto relative = Sub(Rotate(child->motion.orientation, child->motion.angular), Rotate(parent->motion.orientation, parent->motion.angular));
			const float change = std::clamp(a_want - Dot(relative, a_axis), -kCrawlLimbAccel * dt, kCrawlLimbAccel * dt);
			BodyRef parentRef{};
			parentRef.index = a_child.parentIndex;
			parentRef.id = a_child.parentId;
			++a_death.crawlLimbTries;
			if (ApplyJointImpulse(npWorld, a_child, parentRef, Scale(a_axis, change), kCrawlLimbMaxTorque * a_strength * dt)) ++a_death.crawlLimbHits;
		};
		const float omega = 6.2831853f / std::max(kCrawlInterval, 0.1f);  // rad/s of the cycle
		for (const auto& joint : a_death.joints) {
			if (!joint.leg || joint.foot || !joint.side || a_death.Detached(joint.id.m_value)) continue;
			const int j = joint.side == 'L' ? 0 : 1;
			const float k = static_cast<float>(1 - j);  // the opposite arm
			const float phase = cycle + 0.5f * k - std::floor(cycle + 0.5f * k);
			const float wave = std::sin(phase * 6.2831853f) * a_strength;  // + while that arm reaches (draw up), - while it pulls
			if (joint.knee) spin(joint, Scale(right, -1.0f), kCrawlKneeSpeed * wave);
			else spin(joint, Vec3{ 0.0f, 0.0f, j ? 1.0f : -1.0f }, kCrawlHipSpeed * wave);
		}
		int heads = 0;
		for (const auto& s : samples)
			if (s.ref->head && s.ref->constraintId >= 0) ++heads;
		if (heads > 0) {
			const float c = cycle - std::floor(cycle);
			// Turn angle (to the right +) = -turn·sin(2πc): left while the left arm reaches (c < 0.5). Lift bobs twice a
			// cycle: lift·(1 - cos 4πc)/2. Their rates, shared over neck and head bodies.
			const float turnRate = -kCrawlHeadTurn * omega * std::cos(c * 6.2831853f);
			const float liftRate = kCrawlHeadLift * omega * std::sin(c * 12.566371f);
			for (const auto& s : samples) {
				if (!s.ref->head || s.ref->constraintId < 0) continue;
				spin(*s.ref, Vec3{ 0.0f, 0.0f, -1.0f }, turnRate * a_strength / heads);
				spin(*s.ref, right, liftRate * a_strength / heads);
			}
		}
		// The nudge: everything but the legs (they drag) and arms (a planted hand holds) is brought up to the crawl
		// speed along the crawl direction while an arm pulls. Trunk alone could not beat the whole body's friction.
		const float want = kCrawlSpeed * a_strength * pull;
		if (want <= 0.0f) return;
		for (const auto& s : samples) {
			if (s.ref->leg || s.ref->arm) continue;
			const float speed = Dot(s.motion.linear, forward);
			if (speed >= want) continue;
			const auto impulse = Scale(forward, std::min(want - speed, kCrawlAccel * dt) / s.motion.inverseMass);
			RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
			g_linearImpulse(npWorld, s.ref->id.m_value, &nativeImpulse);
		}
	}


	// After the stumble: the body lies there clutching the wound, twitching and now and then rolling, with
	// everything decaying to fully limp over kDyingTime. Twitches are paired joint impulses (spine/legs; the
	// gripping arm is not in the joint list); rolls are a small sideways kick on the trunk.
	void StepWrithe(Death& a_death, RE::Actor* a_actor, float a_now)
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
		// g_mutex is held here (OnWorldUpdate's death loop).
		const bool headOff = g_headless.contains(a_death.formID) ||
		                     std::any_of(a_death.bodies.begin(), a_death.bodies.end(),
								 [&](const BodyRef& a_body) { return a_body.head && a_death.Detached(a_body.id.m_value); });
		if (!a_death.writheStarted) {
			a_death.writheStarted = true;
			a_death.headOffBefore = headOff;
			if (headOff) {
				a_death.roll *= kHeadlessRollScale;
				REX::INFO("DynamicRagdollSystem: {:08X} dying without a head: roll x{:.2f} -> {:.2f}"sv, a_death.formID, kHeadlessRollScale, a_death.roll);
			}
		} else if (kDyingEndOnHeadLoss && headOff && !a_death.headOffBefore) {
			a_death.headCut = true;
			return;
		}
		for (const auto& body : a_death.bodies) {
			if (!FindBody(worldBodies, body)) continue;
			if (a_death.Detached(body.id.m_value)) continue;
			MotionState motion{};
			if (!ReadMotionState(npWorld, worldBodies[body.index], motion)) continue;
			samples.push_back({ &body, motion, BodyPosition(worldBodies[body.index]) });
		}
		if (samples.empty()) return;
		const float dt = ControlDt(a_death.lastStep, a_now);
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
				const bool fire = a_death.burnDown || a_death.burnedToDeath;
				a_death.nextTwitch = a_now + kTwitchDelay;
				a_death.nextRoll = a_now + (fire ? kBurnRollDelay : kTwitchDelay);
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
		// Crawl wins over twitching, but only face down; nothing else plays until that is decided.
		if (a_death.crawlRolled && !a_death.crawlDecided && !CheckCrawl(a_death, npWorld, samples, dt, a_now)) return;
		const float progress = std::clamp((a_now - a_death.downTime) / std::max(a_death.dyingTime, 0.01f), 0.0f, 1.0f);
		const float strength = 1.0f - progress;
		if (a_death.crawling) {
			// Fades over its own time from its start, not over the dying time from the body going down.
			const float crawlStrength = 1.0f - std::clamp((a_now - a_death.crawlStart) / std::max(a_death.crawlTime, 0.01f), 0.0f, 1.0f);
			if (a_now >= a_death.crawlStart) ApplyCrawl(a_death, npWorld, samples, dt, a_now, crawlStrength);
			return;
		}

		// The grip holds fully for the first half, then loosens.
		if (a_death.grip) ApplyGrip(a_death, npWorld, samples, dt, std::min(1.0f, 2.0f * strength));

		const float twitch = a_death.twitch * strength;
		// A twitch is a short burst: the same joint is driven the same way for kTwitchBurst. A single-frame kick
		// was eaten by joint friction and ground contact within a frame or two.
		if (twitch > 0.0f && a_now >= a_death.nextTwitch && !a_death.joints.empty() && g_angularImpulse) {
			a_death.nextTwitch = a_now + (0.15f + 0.35f * Random()) * (1.0f + 1.5f * progress);  // spasms thin out
			// Any spine/leg/arm joint except the arm that clutches the wound.
			char gripSide = 0;
			for (const auto& sample : samples)
				if (sample.ref->id.m_value == a_death.gripHand) gripSide = sample.ref->side;
			a_death.twitchJoint = -1;
			for (int attempt = 0; attempt < 6 && a_death.twitchJoint < 0; ++attempt) {
				const auto pick = static_cast<std::int32_t>(Rng()() % a_death.joints.size());
				const auto& candidate = a_death.joints[static_cast<std::size_t>(pick)];
				if (!(candidate.arm && gripSide && candidate.side == gripSide) && !a_death.Detached(candidate.id.m_value)) a_death.twitchJoint = pick;
			}
			a_death.twitchAxis = Unit(Vec3{ Random() * 2.0f - 1.0f, Random() * 2.0f - 1.0f, Random() * 2.0f - 1.0f });
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
					const float up = joint.leg ? 0.0f : Random() * 1.5f;
					const Vec3 dir = Unit(Vec3{ Random() * 2.0f - 1.0f, Random() * 2.0f - 1.0f, up });
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
			a_death.nextRoll = a_now + (a_death.burnDown ? Random(kBurnRollGapMin, kBurnRollGapMax)
			                                             : kRollGap * (1.0f + Random()));
			a_death.rollEnd = a_now + kRollBurst;
			a_death.rollSign = Random() < 0.5f ? -1.0f : 1.0f;
			a_death.rollApplied = false;
		}
		const bool fire = a_death.burnDown || a_death.burnedToDeath;
		if (roll > 0.0f && a_now < a_death.rollEnd &&
			RollTrunk(npWorld, samples, dt, a_death.rollSign * (fire ? kBurnRollAccel : kRollAccel) * roll, fire ? kBurnRollArms : kDeathRollArms) &&
			!a_death.rollApplied) {
			a_death.rollApplied = true;
			++a_death.rollCount;
		}
	}


	void Hook_KillImpl(RE::Actor* a_actor, RE::Actor* a_attacker, float a_damage, bool a_sendEvent, bool a_ragdollInstant)
	{   //starting point for dead npc
		// Kill moves, sync anims and furniture keep the vanilla death (forcing a ragdoll there cuts paired animations).
		const bool candidate = a_actor && !a_actor->IsPlayerRef() && !a_actor->IsDead(false);
		const bool blacklisted = candidate && IsBlacklisted(a_actor, kDeathBlacklist);  // vanilla death, animation included
		const bool npc = candidate && !blacklisted && !InLockedAnimation(a_actor);
		if (blacklisted) LogSkipOnce(a_actor, "death skipped", "[Death] sBlacklist");
		else if (candidate)
			if (const auto why = LockedReason(a_actor)) LogSkipOnce(a_actor, "death skipped", why);
		// Stumble, grip, twitches, rolls and crawl are built for humanoid skeletons: everything else keeps the vanilla death.
		const bool humanoid = npc && IsHumanoid(a_actor);
		if (npc && !humanoid) LogSkipOnce(a_actor, "death skipped", "not humanoid");
		const bool tracked = humanoid && (kDeathWalkChance > 0.0f || kDeathStumbleChance > 0.0f || kDeathGripChance > 0.0f || kTwitchChance > 0.0f || kCrawlChance > 0.0f);
		// Creatures whose ragdoll fails the joint check keep their death animation: forced into the ragdoll, Nuka-World
		// ants (134% gap) spun forever. For creatures the check only reads the joints. World lock busy: keep the animation.
		const auto creatureJointsFit = [&] {
			auto* world = GetWorld(a_actor);
			auto* npWorld = world ? world->m_worldNP.get() : nullptr;
			if (!npWorld) return false;
			WorldWriteLock lock{ npWorld };
			return lock.held && FitJointsToSkeleton(a_actor, npWorld, true);
		};
		if (npc && kDisableDeathAnimation) {
			if (humanoid || creatureJointsFit()) a_ragdollInstant = true;
			else LogSkipOnce(a_actor, "death ragdoll", "joint check failed; keeps death animation");
		}
		if (tracked && IsBurning(a_actor) && !IsBlacklisted(a_actor, kBurnBlacklist)) {
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
		if (tracked && IsReallyDead(a_actor)) {
			g_pendingDeaths.push_back(a_actor->GetFormID()); //put dead npc into queue to process it
		} else if (tracked) {
			if (a_actor->IsDead(false)) LogSkipOnce(a_actor, "death skipped", std::format("essential, downed (life state {})", static_cast<std::uint32_t>(a_actor->lifeState)));
			g_deathMoveVel.erase(a_actor->GetFormID());  // did not die after all (essential): no leftover entry
			std::erase(g_burningDeaths, a_actor->GetFormID());
		}
	}


	void StartPendingDeaths(float a_now) //for each npc in g_pendingDeaths we decide what to do with the body
	{
		std::vector<RE::TESFormID> pending;
		{
			std::scoped_lock lock{ g_mutex };
			pending.swap(g_pendingDeaths);
		}
		for (const auto formID : pending) {
			const auto actor = RE::TESForm::GetFormByID<RE::Actor>(formID);
			if (!actor || !actor->Get3D() || !GetWorld(actor)) continue;
			const auto pick = [&](const Range& a_range) { return Random(a_range.min, a_range.max); };
			Death death{};
			death.formID = formID;
			// Walk (wanders on its feet) takes over from the stumble; a head shot can still skip both (below, once the hit is known).
			death.walk = Random() < kDeathWalkChance;
			death.stumble = !death.walk && Random() < kDeathStumbleChance;
			death.grip = Random() < kDeathGripChance;
			const bool moves = Random() < kTwitchChance;
			// Burned to death (on fire at KillImpl): its own, stronger writhing, whatever the twitch chance rolled.
			bool burned = false;
			{
				std::scoped_lock lock{ g_mutex };
				if (const auto it = std::find(g_burningDeaths.begin(), g_burningDeaths.end(), formID); it != g_burningDeaths.end()) {
					g_burningDeaths.erase(it);
					burned = kBurnDeathTwitchMax > 0.0f || kBurnDeathRollMax > 0.0f;
				}
			}
			death.crawlRolled = !burned && Random() < kCrawlChance && !IsBlacklisted(actor, kCrawlBlacklist);  // burned deaths writhe instead
			if (!death.walk && !death.stumble && !death.grip && !moves && !burned && !death.crawlRolled) {
				REX::INFO("DynamicRagdollSystem: {:08X} death: no walk, stumble, grip or twitch rolled; vanilla ragdoll"sv, formID);
				continue;
			}
			death.support = pick(kDeathSupport);
			death.upright = pick(kDeathUpright);
			death.braking = pick(kDeathBraking);
			death.dyingTime = (death.grip || moves || burned || death.crawlRolled) ? pick(kDyingTime) : 0.0f;
			const bool powerArmor = RE::PowerArmor::ActorInPowerArmor(*actor);
			death.twitch = moves ? pick(powerArmor ? kPowerArmorTwitchStrength : kTwitchStrength) : 0.0f;
			death.roll = moves ? pick(powerArmor ? kPowerArmorRollStrength : kRollStrength) : 0.0f;
			if (burned) {
				death.burnedToDeath = true;
				death.twitch = Random(kBurnDeathTwitchMin, kBurnDeathTwitchMax);
				death.roll = Random(kBurnDeathRollMin, kBurnDeathRollMax);
				death.dyingTime = Random(kBurnDeathTimeMin, kBurnDeathTimeMax);
				REX::INFO("DynamicRagdollSystem: {:08X} burned to death: twitch {:.2f} roll {:.2f} for {:.1f}s"sv, formID, death.twitch,
					death.roll, death.dyingTime);
			}
			death.flail = pick(kFlailStrength);
			death.start = a_now;
			// Without a stumble, StepDeathStumble still runs until the bodies are scanned (it then ends the stumble).
			death.end = a_now + (death.walk ? pick(kDeathWalkTime) : death.stumble ? pick(kDeathStumbleTime) : 5.0f);
			REX::INFO("DynamicRagdollSystem: {:08X} death rolled: walk {} stumble {} ({:.2f}s, support {:.2f}, upright {:.2f}, braking {:.2f}) grip {} dying {:.2f}s twitch {:.2f} roll {:.2f} crawl {}"sv,
				formID, death.walk, death.stumble, death.end - a_now, death.support, death.upright, death.braking, death.grip, death.dyingTime,
				death.twitch, death.roll, death.crawlRolled);
			std::scoped_lock lock{ g_mutex };
			if (const auto it = g_lastHit.find(formID); it != g_lastHit.end() && a_now - it->second.time < kLastHitWindow) {
				death.hitDir = it->second.dir;
				death.hitBody = it->second.hitBody;
				death.pushScale = it->second.pushScale;
				death.weapon = it->second.weapon;
				death.damage = it->second.damage;
				REX::INFO("DynamicRagdollSystem: {:08X} killing hit: flags {:08X} limb {}"sv, formID, it->second.flags, it->second.limb);
				// Head limbs (Head1, Eye1, Head2, Brain). A fatal head shot often loses the head on death without any
				// dismember flag on the hit, so it rules out the crawl by itself.
				const auto limb = it->second.limb;
				death.headShotKill = limb == 1 || limb == 2 || limb == 5 || limb == 0x10;
				// Rolled first in priority: a skipped head shot neither walks nor stumbles.
				if (death.headShotKill && (death.walk || death.stumble) && Random() < kHeadshotSkipStumbleChance) {
					REX::INFO("DynamicRagdollSystem: {:08X} head shot: {} skipped"sv, formID, death.walk ? "walk" : "stumble");
					death.walk = false;
					death.stumble = false;
					death.end = a_now + 5.0f;  // as a death rolled without a stumble
				}
				// Explosion deaths are flung: no balance help (support/upright/braking), the blast push, and the limbs
				// flail while airborne. The stumble machinery still runs for the joints, grip and flailing.
				if (it->second.explosion && Random() >= kExplosionStumbleChance) {
					if (!death.stumble) death.end = a_now + pick(kDeathStumbleTime);
					death.walk = false;  // flung bodies don't walk
					death.stumble = true;
					death.support = 0.0f;
					death.upright = 0.0f;
					death.braking = 0.0f;
					death.explosionPush = pick(kExplosionPush);
					death.flung = true;
					REX::INFO("DynamicRagdollSystem: {:08X} explosion death: flung at {:.1f} (lift {:.2f})"sv, formID, death.explosionPush, kExplosionLift);
				} else if ((kDeathFlingDamage > 0.0f && it->second.damage >= kDeathFlingDamage) ||
						   (kDeathFlingPellets > 0 && it->second.burstHits >= kDeathFlingPellets)) {
					// Heavy hits (e.g. a Gauss rifle) blow the body away: same as an explosion but with the weapon's own
					// knockback kept in full instead of a blast push.
					if (!death.stumble) death.end = a_now + pick(kDeathStumbleTime);
					death.walk = false;  // flung bodies don't walk
					death.stumble = true;
					death.support = 0.0f;
					death.upright = 0.0f;
					death.braking = 0.0f;
					death.flung = true;
					REX::INFO("DynamicRagdollSystem: {:08X} heavy hit ({:.0f} damage, {} pellets, '{}'): flung"sv, formID, it->second.damage,
						it->second.burstHits, it->second.weapon);
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
