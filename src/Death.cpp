#include "Common.h"
//
// dies -> Hook_KillImpl -> remember pre-death velocity -> not in bleedout anim (IsReallyDead) -> add to death queue (g_pendingDeaths)
// -> world update -> create controller for dead body (StartPendingDeaths) -> manipulate body for target anim
//
namespace DynamicRagdollSystem
{

	// The trunk for steering the hands: pelvis and chest, and the chest's axes from the shoulders (upper arms: arm bodies
	// hanging off a non-arm body). No forward without both shoulders.
	struct TrunkAxes
	{
		const DeathSample* low{};
		const DeathSample* high{};
		Vec3               up{}, forward{}, right{};
	};

	static TrunkAxes TrunkFrame(const std::vector<DeathSample>& a_samples)
	{
		TrunkAxes t;
		std::tie(t.low, t.high) = TrunkEnds(a_samples);
		const DeathSample* shoulder[2]{};  // left, right upper arm
		for (const auto& s : a_samples) {
			if (!s.ref->arm || !s.ref->side) continue;
			for (const auto& parent : a_samples)
				if (parent.ref->id.m_value == s.ref->parentId.m_value && !parent.ref->arm) shoulder[s.ref->side == 'R'] = &s;
		}
		if (t.low && t.high && t.low != t.high && shoulder[0] && shoulder[1]) {
			t.up = Unit(Sub(t.high->pos, t.low->pos));
			t.forward = Unit(Cross(t.up, Unit(Sub(shoulder[1]->pos, shoulder[0]->pos))));
			t.right = Cross(t.forward, t.up);
		}
		return t;
	}

	// Where to pull a hand so it gets to a_target round the trunk (RagdollLab sim: Death::Route). Pulled straight, a hand
	// behind the trunk dragged the arm into the back and stuck there, and one reaching across the body (the other shoulder)
	// pressed into the belly and stuck. Behind the chest: straight out to its own side past the hips, then round in front
	// of the target. Across: out in front of the trunk first, then over. a_behind (optional): 0..1, how far the hand is
	// behind the chest. a_round false: no behind-the-chest detour (the old stumble's grip, before 4b97ad8).
	static Vec3 Route(const TrunkAxes& a_trunk, const DeathSample& a_hand, Vec3 a_target, float* a_behind = nullptr, bool a_round = true)
	{
		if (a_behind) *a_behind = 0.0f;
		if (Length(a_trunk.forward) < 0.5f) return a_target;
		const Vec3  forward = a_trunk.forward, out = Scale(a_trunk.right, a_hand.ref->side == 'R' ? 1.0f : -1.0f);
		const Vec3  hand = Sub(a_hand.pos, a_trunk.high->pos);
		const float across = std::clamp(std::min(Dot(hand, out), -Dot(Sub(a_target, a_trunk.high->pos), out)) / kGripClear, 0.0f, 1.0f);
		const float ahead = std::clamp((Dot(hand, forward) - kGripAround + kGripClear) / kGripClear, 0.0f, 1.0f);
		const Vec3  front = Add(a_hand.pos, Scale(forward, kGripAround));  // a full kGripAround away, as sideways below
		const Vec3  past = Add(front, Scale(Sub(Add(a_target, Scale(forward, kGripAround)), front), ahead));
		const Vec3  target = Add(a_target, Scale(Sub(past, a_target), across));
		if (!a_round) return target;
		// Still within the body's width (behind the buttocks, the small of the back): straight out to the side first, or
		// the way round the front still runs up the back. A hand that only lags a moment behind the chest goes straight.
		const float detour = std::clamp((-Dot(hand, forward) - kGripLag) / kGripClear, 0.0f, 1.0f);
		const float aside = std::clamp((Dot(Sub(a_hand.pos, a_trunk.low->pos), out) - kGripOut + kGripClear) / kGripClear, 0.0f, 1.0f);
		const Vec3  sideways = Add(a_hand.pos, Scale(out, kGripOut));  // a full kGripOut away: nearer pulls too weakly to free it
		const Vec3  around = Add(a_target, Add(Scale(forward, kGripAround), Scale(out, kGripOut)));
		if (a_behind) *a_behind = detour;
		return Add(target, Scale(Sub(Add(sideways, Scale(Sub(around, sideways), aside)), target), detour));
	}

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
					// An arm can't clutch its own shoulder or upper arm: a shot arm is clutched by the other hand.
					if (!s.ref->hand || s.ref->id.m_value == woundId || (wound->ref->arm && wound->ref->side && s.ref->side == wound->ref->side)) continue;
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
					const bool dropElbow = a_stumble.stagger && parent && !parent->ref->arm && hand->pos.z > wound->pos.z;  // death stagger only
					const auto lift = Scale(Vec3{ g.x, g.y, g.z }, -m * dt * a_strength * (dropElbow ? kGripUpperArmCarry : 1.0f));
					RE::hkVector4f liftImpulse{ lift.x, lift.y, lift.z, 0.0f };
					g_linearImpulse(npWorld, s.ref->id.m_value, &liftImpulse);
					woundImpulse = Sub(woundImpulse, lift);
				}
				// The front of the wounded part, not its centre: pulled at the centre, a hand that reached the body high up
				// pressed straight into it and friction held it there (under the chin, on the collarbone). Routed round the
				// trunk (behind the back, across the body).
				const TrunkAxes trunk = TrunkFrame(samples);
				Capsule capsule;
				const auto* woundBody = FindBody(npWorld->m_bodyManager.m_bodies, *wound->ref);
				const float radius = woundBody && ReadCapsule(*woundBody, capsule) ? capsule.radius : 0.0f;
				const Vec3  target = Route(trunk, *hand, Add(wound->pos, Scale(trunk.forward, radius)), nullptr, a_stumble.stagger);
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
				// Equal and opposite. A shot arm: on the chest, which the clutching arm hangs from (on the shot arm, the other
				// arm's weight dragged it down).
				RE::hkVector4f toWound{ woundImpulse.x, woundImpulse.y, woundImpulse.z, 0.0f };
				g_linearImpulse(npWorld, wound->ref->arm && trunk.high ? trunk.high->ref->id.m_value : woundId, &toWound);
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
		const TrunkAxes trunk = TrunkFrame(samples);
		const DeathSample* low = trunk.low;  // pelvis / SPINE1
		const DeathSample* high = trunk.high;  // chest
		if (!low || !high || low == high) return;
		const auto& g = npWorld->m_gravity;
		// A free hand thrown behind the back by the hit and the stagger came to rest there, the trunk leaning back over it,
		// and stayed, stiff. Eased out to its side and in front of the hips the way the grip goes round, and let go once
		// out, so the arm swings loose. Handed over to the brace as it comes in (which routes it the same way).
		Vec3 easeImpulse{};
		for (const auto& hand : samples) {
			if (!hand.ref->hand || hand.ref->id.m_value == a_death.gripHand) continue;
			float       behind = 0.0f;
			const auto  target = Route(trunk, hand, low->pos, &behind);
			const float ease = behind * (1.0f - std::min(amount, 1.0f));
			if (ease <= 0.0f) continue;
			float armMass = 0.0f;
			for (const auto& s : samples)
				if (s.ref->arm && s.ref->side == hand.ref->side) armMass += 1.0f / s.motion.inverseMass;
			const auto relVel = Sub(hand.motion.linear, high->motion.linear);
			Vec3 deltaV = Scale(Sub(Scale(Sub(target, hand.pos), kGripKp), Scale(relVel, kGripKd)), dt * ease);
			const float n = Length(deltaV), cap = kGripMaxAccel * dt;
			if (!std::isfinite(n) || n < 1.0e-4f) continue;
			if (n > cap) deltaV = Scale(deltaV, cap / n);
			const auto impulse = Scale(deltaV, armMass);
			RE::hkVector4f nativeImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
			g_linearImpulse(npWorld, hand.ref->id.m_value, &nativeImpulse);
			easeImpulse = Sub(easeImpulse, impulse);
		}
		if (Length(easeImpulse) > 0.0f) {
			RE::hkVector4f toChest{ easeImpulse.x, easeImpulse.y, easeImpulse.z, 0.0f };
			g_linearImpulse(npWorld, high->ref->id.m_value, &toChest);
		}
		if (amount <= 1.0e-4f) return;
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
		// Falling backward: in front all the same (mirrored). Guarding the way it fell pulled the hands up behind the head,
		// crossed over (left and right flip with it).
		if (const float back = Dot(forward, trunk.forward); back < 0.0f) forward = Sub(forward, Scale(trunk.forward, 2.0f * back));
		const auto right = Cross(forward, up);
		const auto guardPoint = [&](float a_up, float a_forward, float a_side) {
			auto p = Add(Add(high->pos, Scale(up, a_up)), Add(Scale(forward, a_forward), Scale(right, a_side)));
			p.z = std::max(p.z, a_ground + 0.05f);
			return p;
		};
		a_death.braceAmount = amount;
		float offSum = 0.0f;
		int offCount = 0;
		Vec3 chestImpulse{};
		for (const auto& hand : samples) {
			if (!hand.ref->hand || hand.ref->id.m_value == a_death.gripHand) continue;
			const float sideSign = hand.ref->side == 'L' ? -1.0f : 1.0f;
			float behind = 0.0f;
			const auto target = Route(trunk, hand, guardPoint(kGuardHandUp, kGuardHandForward, kGuardHandSide * sideSign), &behind);
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
			// Not while the hand is still coming round from behind the back: Route steers only the hand.
			if (forearm && behind <= 0.0f) drive(*forearm, guardPoint(kGuardElbowUp, kGuardElbowForward, kGuardElbowSide * sideSign), 0.5f * armMass);
		}
		RE::hkVector4f toChest{ chestImpulse.x, chestImpulse.y, chestImpulse.z, 0.0f };
		g_linearImpulse(npWorld, high->ref->id.m_value, &toChest);
		a_death.braceOff = offCount ? offSum / static_cast<float>(offCount) : -1.0f;
		if (!a_death.braceLogged) {
			a_death.braceLogged = true;
			REX::INFO("DynamicRagdollSystem: {:08X} brace: arms guard the head (lean {:.2f}, strength {:.2f})"sv, a_death.formID, a_lean, amount);
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
		const bool tracked = humanoid && (kDeathWalkChance > 0.0f || kDeathStaggerChance > 0.0f || kDeathStumbleChance > 0.0f || kDeathGripChance > 0.0f || kTwitchChance > 0.0f || kCrawlChance > 0.0f);
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
			// Walk (wanders on its feet), else the stagger, else the stumble; a head shot can still skip them (below, once the hit is known).
			// The stagger runs as a stumble (stumble timers, grip, brace, dying phase after it) with Death::stagger set.
			// A fatal head shot can cut the head off on death with no dismember flag on the hit (flags 0x20 only), so
			// g_headless misses it; the engine's pending head hide (set until the head is hidden) catches those.
			auto* middleHigh = actor->currentProcess ? actor->currentProcess->middleHigh : nullptr;
			const bool headHide = middleHigh && middleHigh->headDeferredHideLimb;
			bool headless;
			{
				std::scoped_lock lock{ g_mutex };
				headless = headHide || g_headless.contains(formID);
			}
			if (kDeathWalkHeadlessOnly)
				REX::INFO("DynamicRagdollSystem: {:08X} death walk headless check: {} (pending head hide {})"sv, formID, headless, headHide);
			death.walk = (!kDeathWalkHeadlessOnly || headless) && Random() < kDeathWalkChance;
			death.stagger = !death.walk && Random() < kDeathStaggerChance;
			death.stumble = death.stagger || (!death.walk && Random() < kDeathStumbleChance);
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
				REX::INFO("DynamicRagdollSystem: {:08X} death: no walk, stagger, stumble, grip or twitch rolled; vanilla ragdoll"sv, formID);
				continue;
			}
			death.support = pick(kDeathSupport);
			death.upright = pick(kDeathUpright);
			death.braking = pick(kDeathBraking);
			if (death.stagger) {
				death.tip = pick(kDeathStaggerTip);
				death.staggerUpright = pick(kStaggerUpright);
				death.staggerBraking = pick(kDeathStaggerBraking);
			}
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
			REX::INFO("DynamicRagdollSystem: {:08X} death rolled: walk {} stagger {} stumble {} ({:.2f}s, support {:.2f}, upright {:.2f}, braking {:.2f}) grip {} dying {:.2f}s twitch {:.2f} roll {:.2f} crawl {}"sv,
				formID, death.walk, death.stagger, death.stumble, death.end - a_now, death.support, death.upright, death.braking, death.grip, death.dyingTime,
				death.twitch, death.roll, death.crawlRolled);
			if (death.stagger)
				REX::INFO("DynamicRagdollSystem: {:08X} death stagger rolled: tip {:.2f} upright {:.2f} braking {:.2f}"sv, formID, death.tip,
					death.staggerUpright, death.staggerBraking);
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
				// Rolled first in priority: a skipped head shot neither walks, staggers nor stumbles.
				// A headless-only walk is a head shot by definition: the skip would only cancel it.
				if (death.headShotKill && !(death.walk && kDeathWalkHeadlessOnly) && (death.walk || death.stumble) &&
					Random() < kHeadshotSkipStumbleChance) {
					REX::INFO("DynamicRagdollSystem: {:08X} head shot: {} skipped"sv, formID, death.walk ? "walk" : death.stagger ? "stagger" : "stumble");
					death.walk = false;
					death.stagger = false;
					death.stumble = false;
					death.end = a_now + 5.0f;  // as a death rolled without a stumble
				}
				// Explosion deaths are flung: no balance help (support/upright/braking), the blast push, and the limbs
				// flail while airborne. The stumble machinery still runs for the joints, grip and flailing.
				if (it->second.explosion && Random() >= kExplosionStumbleChance) {
					if (!death.stumble) death.end = a_now + pick(kDeathStumbleTime);
					death.walk = false;  // flung bodies don't walk
					death.stagger = false;  // ... or stagger: the stumble's controller, as before the stagger
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
					death.stagger = false;  // ... or stagger: the stumble's controller, as before the stagger
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
