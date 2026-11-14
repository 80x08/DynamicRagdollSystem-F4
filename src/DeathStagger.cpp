#include "Walker.h"

// Death stagger ([DeathStagger] fDeathStaggerChance): the walker (Walker.h) with no goal of its own, staggering under the
// hit (RagdollLab sim/src/Puppet.cpp: BeginStagger, Stagger). StepDeathStumble (DeathStumble.cpp) drives it and keeps the
// push, grip, brace and flail.

namespace DynamicRagdollSystem
{
	using namespace walker;

	namespace
	{
		// Stagger (the sim's Puppet constants; the ones most likely to need tuning are [DeathStagger] settings). Lowest
		// pelvis (fraction of standing) a step's reach allows for; strength below which it no longer steps; m the capture
		// point may be off the feet before a step; m the centre of pressure may be off the line between the feet (half a
		// sole); ground friction; pelvis height (fraction of standing) once the legs have given way; m the weight may be
		// off the feet before the lift starts fading / is gone; m a foot may be from under its hip before it steps back in.
		constexpr float kStaggerLow = 0.7f, kStaggerWeakest = 0.2f, kStaggerTrigger = 0.1f, kSole = 0.08f;
		constexpr float kStaggerFriction = 0.6f, kBuckled = 0.4f, kStaggerBalanced = 0.2f, kStaggerToppled = 0.3f, kStaggerWide = 0.35f;
		// m a shot leg's foot is lifted to fold its knee; pelvis height (fraction of standing) below which, and m from under
		// its hip beyond which, a leg left out folds the same way.
		constexpr float kHurtLift = 0.12f, kStaggerSplit = 0.7f, kStaggerFar = 0.5f;

		// The walker with no goal of its own (RagdollLab sim: Puppet::Stagger): the hit's momentum carries the body and the
		// feet step to catch it. The trunk is held up and upright through the planted feet as in Walk, but pushed sideways
		// only as hard as legs standing there can push: like an inverted pendulum, the ground force points from the centre
		// of pressure (somewhere under the planted feet) to the weight, so it can brake the body only while the feet are
		// ahead of it. Each step therefore lands where the weight is heading (its capture point, as it will be at
		// touchdown), which is what slows it down. a_strength scales every effort, a_legs is the share of the weight the
		// legs still carry, c.stiffness the trunk springs, a_brake (1/s) how hard it tries to stop, a_tip how far the centre
		// of pressure lags behind the weight the way it is going (it tips over). Heights: s.ground under the trunk,
		// s.footGround under each foot, s.landing under the spot. False while no foot is down.
		bool Stagger(Ctx& c, float a_strength, float a_legs, float a_brake, float a_tip)
		{
			auto&       s = c.s;
			c.strength = a_strength;
			c.feetOnly = c.stiffness < 1.0f;
			const auto ahead = Flat(Rotate(c.BodyFrame(), kY));  // the trunk faces where the pelvis now faces
			if (Length(ahead) > 0.1f) s.yaw = YawOf(ahead);
			const auto facing = Yaw(s.yaw);
			const auto right = Rotate(facing, kX);

			// The weight (trunk and arms; the legs stand on the ground themselves) and its capture point.
			Vec3  com{}, vel{};
			float mass = 0.0f, trunkMass = 0.0f;
			const auto weigh = [&](std::uint32_t a_id, bool a_trunk) {
				MotionState m{};
				if (!c.Read(a_id, m)) return;
				const float w = 1.0f / m.inverseMass;
				com = Add(com, Scale(c.Com(a_id), w));
				vel = Add(vel, Scale(m.linear, w));
				mass += w;
				if (a_trunk) trunkMass += w;
			};
			for (const auto& t : s.trunk) weigh(t.id, true);
			for (const auto a : s.arms) weigh(a, false);
			if (mass <= 0.0f || trunkMass <= 0.0f) return false;
			com = Scale(com, 1.0f / mass);
			vel = Scale(vel, 1.0f / mass);
			const float g = std::max(c.gravity, 1.0f), height = std::max(com.z - s.ground, 0.3f), omega = std::sqrt(g / height);
			const auto  capture = Add(Flat(com), Scale(Flat(vel), 1.0f / omega));
			const float speed = Length(Flat(vel));

			bool down[2]{};
			Vec3 foot[2];
			for (int i = 0; i < 2; ++i) {
				foot[i] = c.Pos(s.feet[i].id);
				down[i] = foot[i].z - (s.footGround[i] + s.feet[i].offset.z) < kContact;
				foot[i] = Flat(foot[i]);
			}
			// A step is over once the foot is down and its time is up, or once the other foot has left the ground (rolled
			// off its toes): the weight is on it now.
			if (s.swing >= 0 && down[s.swing] && (s.time - s.swingStart >= s.swingTime || !down[1 - s.swing])) {
				s.lastSwing = s.swing;
				s.landed = s.time;
				s.swing = -1;
			}
			// Where a step lands: the capture point as it will be after a_left more seconds on the other foot (it runs away
			// from that foot meanwhile, so a faster body takes a longer step), on the foot's own side. Shorter as strength
			// fades (from where the step began: measured from the moving foot, it chased the weight into a lunge), never
			// beyond the leg's reach from where the hip will be by then, with the pelvis halfway down to kStaggerLow (reach
			// measured with the pelvis already that low left the foot hanging, leg straight, short of a spot it could not get
			// down to). Landing height: the step ray's ground, else the foot's own.
			const auto  pelvisPos = c.Pos(s.trunk[0].id);
			const auto  pelvis = Flat(pelvisPos);
			const float lowest = s.standHeight * kStaggerLow, reachHeight = 0.5f * (std::max(pelvisPos.z - s.ground, lowest) + lowest);
			const auto  spotFor = [&](int a_foot, float a_left) {
				const auto& f = s.feet[a_foot];
				const auto  from = foot[1 - a_foot], start = a_foot == s.swing ? Flat(s.swingFrom) : foot[a_foot];
				auto        to = Add(Add(from, Scale(Sub(capture, from), std::exp(omega * a_left))), Scale(right, f.offset.x));
				to = Add(start, Clamp(Sub(to, start), kDeathStaggerStep * a_strength * (1.0f + 0.5f * speed)));  // faster: longer (to the leg's reach)
				const auto  hip = Add(Add(pelvis, Scale(Flat(vel), a_left)), Scale(right, f.offset.x));
				const float land = a_foot == s.swing && s.landingValid ? s.landing : s.footGround[a_foot];
				const float drop = s.ground + reachHeight - (land + f.offset.z), leg = kStraight * f.leg;
				to = Add(hip, Clamp(Sub(to, hip), std::sqrt(std::max(0.0f, leg * leg - drop * drop))));
				to.z = land + f.offset.z;
				return OwnSide(c, to, a_foot, right);
			};
			// A step starts when the capture point leaves the feet, or at once for a foot the hit kicked up. Otherwise a foot
			// left far from under its hip steps back in: the body stopped over spread feet stood in a straddle and sank into a
			// split. A shot leg folded up on purpose (below) counts as standing here, so the feet still alternate.
			const bool stands[2]{ down[0] || s.hurt == 0, down[1] || s.hurt == 1 };
			if (s.swing < 0 && a_strength > kStaggerWeakest) {
				int pick = -1;
				if (stands[0] != stands[1]) pick = stands[0] ? 1 : 0;
				else if (stands[0] && s.time - s.landed >= 0.25f * kWalkStepTime) {
					const auto  ab = Sub(foot[1], foot[0]);
					const float t = Dot(ab, ab) > 1.0e-6f ? std::clamp(Dot(Sub(capture, foot[0]), ab) / Dot(ab, ab), 0.0f, 1.0f) : 0.0f;
					const float out[2]{ Length(Sub(Sub(foot[0], pelvis), Scale(right, s.feet[0].offset.x))),
						Length(Sub(Sub(foot[1], pelvis), Scale(right, s.feet[1].offset.x))) };
					if (Length(Sub(capture, Add(foot[0], Scale(ab, t)))) > kStaggerTrigger * (2.0f - a_strength)) {  // weaker: reacts later
						// Feet alternate, as in Walk; the first step goes with the foot that has farther to go. (Always the
						// farther: one leg stepped again and again while the other trailed far out, straight.)
						const float time = kWalkStepTime / (1.0f + 0.5f * speed);
						pick = s.lastSwing >= 0 ? 1 - s.lastSwing :
						       Length(Flat(Sub(spotFor(0, time), foot[0]))) > Length(Flat(Sub(spotFor(1, time), foot[1]))) ? 0 : 1;
					} else if (std::max(out[0], out[1]) > kStaggerWide) {
						pick = out[0] > out[1] ? 0 : 1;
					}
				}
				if (pick >= 0) {
					s.swing = pick;
					s.swingStart = s.time;
					s.swingTime = kWalkStepTime / ((1.0f + 0.5f * speed) * (0.5f + 0.5f * a_strength));  // quicker when moving fast, slower when weak
					s.swingFrom = c.Pos(s.feet[pick].id);
					s.landingValid = false;  // the step ray is cast for the new spot next frame
				}
			}
			std::vector<std::uint32_t> planted;
			for (int i = 0; i < 2; ++i)
				if (i != s.swing && down[i]) planted.push_back(s.feet[i].id);
			// A shot leg bears no weight while the other foot is down: the body lurches over onto it.
			if (planted.size() == 2 && s.hurt >= 0) std::erase(planted, s.feet[s.hurt].id);
			if (planted.empty()) return false;  // airborne: limp until a foot comes down
			// And it gives: the knee folds and the foot comes up where it is. Left on the floor, the straight leg propped the
			// body up as a strut and it stood on it. (Then it is put down again as a step, and gives again: a hobble.) So does
			// a leg left far out once the body is sinking: held straight, it slid out into a split.
			const bool sinking = pelvisPos.z - s.ground < kStaggerSplit * s.standHeight;
			for (int i = 0; i < 2; ++i) {
				const auto  under = Add(pelvis, Scale(right, s.feet[i].offset.x));
				const bool  shot = i == s.hurt && down[i] && planted[0] != s.feet[i].id;
				const bool  left = sinking && down[i] && Length(Sub(foot[i], under)) > kStaggerFar;
				if (i == s.swing || !(shot || left)) continue;
				auto ankle = left ? Scale(Add(foot[i], under), 0.5f) : foot[i];  // a far one is drawn in, or it lifts straight
				ankle.z = s.footGround[i] + s.feet[i].offset.z + kHurtLift;
				c.Push(s.trunk[0].id, Scale(DriveLeg(c, i, ankle, Rotate(facing, kY)), -1.0f));
			}
			if (s.swing >= 0) {
				// Sped up if the body has: the step that starts with the hit is timed before the hit's push lands.
				s.swingTime = std::min(s.swingTime, kWalkStepTime / ((1.0f + 0.5f * speed) * (0.5f + 0.5f * a_strength)));
				s.spot = spotFor(s.swing, std::max(0.0f, s.swingTime - (s.time - s.swingStart)));
				Swing(c, s.spot, s.swingTime, facing);
			}

			// Lift: only while the weight is over the feet (mid-step: between the standing foot and where the stepping one
			// lands), times a_legs. Height: as high as a nearly straight leg reaches from each planted foot (a pendulum's arc),
			// sinking toward kBuckled as a_legs fades (the knees give way).
			const auto  a = Flat(c.Pos(planted.front())), b = s.swing >= 0 ? Flat(s.spot) : Flat(c.Pos(planted.back())), ab = Sub(b, a);
			const float along = Dot(ab, ab) > 1.0e-6f ? std::clamp(Dot(Sub(Flat(com), a), ab) / Dot(ab, ab), 0.0f, 1.0f) : 0.0f;
			c.support = a_legs * (1.0f - Smooth((Length(Sub(Flat(com), Add(a, Scale(ab, along)))) - kStaggerBalanced) / kStaggerToppled));
			if (planted.size() == 1 && s.hurt >= 0 && planted[0] == s.feet[s.hurt].id) c.support *= kDeathStaggerHurtLeg;  // the shot leg buckles under it
			float reach = s.standHeight;
			for (int i = 0; i < 2; ++i) {
				if (i == s.swing || !down[i]) continue;
				const float d = Length(Add(Sub(pelvis, foot[i]), Scale(right, s.feet[i].offset.x))), leg = kStraight * s.feet[i].leg;
				reach = std::min(reach, s.footGround[i] - s.ground + s.feet[i].offset.z + std::sqrt(std::max(0.0f, leg * leg - d * d)));
			}
			const float buckled = s.standHeight * kBuckled;
			const Vec3  base{ pelvis.x, pelvis.y, s.ground + buckled + (std::max(reach, lowest) - buckled) * a_legs };

			// Sideways: the ground pushes the weight from a centre of pressure under the planted feet, which the legs pick to
			// brake (a_brake) as far as the feet allow, with the share of the weight they carry; within friction.
			// The legs fail to hold it: the centre of pressure lags a_tip behind the weight the way it was going (and toward
			// a shot leg), so it never settles upright: it tips over and the steps have to chase it.
			// (a shot leg's side counts double: thrown onto the good leg, the two cancelled and it stood on it)
			const auto toHurt = s.hurt >= 0 ? Scale(right, s.feet[s.hurt].offset.x < s.feet[1 - s.hurt].offset.x ? -2.0f : 2.0f) : Vec3{};
			const auto tip = UnitOr(Add(s.fallDir, toHurt), Vec3{});
			const auto want = Sub(Add(Flat(com), Scale(Flat(vel), a_brake * a_strength / (omega * omega))), Scale(tip, a_tip));
			Vec3       cop = a;
			if (planted.size() == 2) {
				const auto  pq = Sub(Flat(c.Pos(planted.back())), a);
				const float t = Dot(pq, pq) > 1.0e-6f ? std::clamp(Dot(Sub(want, a), pq) / Dot(pq, pq), 0.0f, 1.0f) : 0.0f;
				cop = Add(a, Scale(pq, t));
			}
			cop = Add(cop, Clamp(Sub(want, cop), kSole));
			const auto push = Clamp(Scale(Sub(Flat(com), cop), omega * omega * c.support), kStaggerFriction * g * c.support);
			// The trunk springs only keep its shape and hold it up (aimed at where the pelvis is, moving as the weight moves);
			// the net sideways push is then made exactly the ground's. The trunk slumps toward the tip as well: held straight
			// over feet set across the tip, the centre of pressure (half a sole of room) could not tip it and it just stood.
			const auto slump = Length(tip) > 0.5f ? AxisAngle(Unit(Cross(kZ, tip)), kDeathStaggerSlump * a_tip) : Yaw(0.0f);
			auto       impulse = HoldTrunk(c, base, Multiply(slump, facing), Vec3{ vel.x, vel.y, 0.0f });
			const auto extra = Scale(Sub(Scale(push, mass * c.dt), Flat(impulse)), 1.0f / trunkMass);
			for (const auto& t : s.trunk) {
				MotionState m{};
				if (!c.Read(t.id, m)) continue;
				c.AddVelocity(t.id, m, extra);
				impulse = Add(impulse, Scale(extra, 1.0f / m.inverseMass));
			}
			Brace(c, impulse, planted);
			HoldStance(c, down);
			return true;
		}
	}

	WalkResult StepDeathStagger(Death& a_death, RE::Actor* a_actor, RE::hknpBSWorld* a_world, float a_dt, float a_now, float a_fade, float a_help)
	{
		if (!g_linearImpulse || a_dt <= 0.0f) return WalkResult::kUnable;
		auto&                       worldBodies = a_world->m_bodyManager.m_bodies;
		std::vector<const BodyRef*> refs;
		Ctx                         c{ a_world, g_walkers[a_death.formID], {}, a_dt, -a_world->m_gravity.z };
		for (const auto& body : a_death.bodies) {
			const auto* b = FindBody(worldBodies, body);
			if (!b || a_death.Detached(body.id.m_value)) continue;
			refs.push_back(&body);
			c.bodies[body.id.m_value] = b;
		}
		const auto ref = [&](std::uint32_t a_id) -> const BodyRef* {
			for (const auto& body : a_death.bodies)
				if (body.id.m_value == a_id) return &body;
			return nullptr;
		};
		auto& s = c.s;
		if (s.deathStart != a_death.start || s.trunk.empty()) {
			s = Walker{};
			s.ground = a_death.groundCaptured ? a_death.groundAtDeath : a_death.feetCaptured.z - 0.1f;
			// Facing the way the hips face (left to right across the thighs, in the death pose), not the actor's heading.
			Vec3 hips{};
			for (const auto& t : a_death.targets)
				if (const auto* r = ref(t.id); r && r->leg && !r->knee && !r->foot && r->side) hips = Add(hips, Scale(Flat(t.pos), r->side == 'R' ? 1.0f : -1.0f));
			const float yaw = Length(hips) > 0.05f ? std::atan2(-hips.y, hips.x) : a_actor->data.angle.z;
			if (!Setup(a_death, yaw, refs, c, s)) {
				g_walkers.erase(a_death.formID);
				REX::INFO("DynamicRagdollSystem: {:08X} death stagger: skeleton not usable (needs a pelvis and two legs); balance stumble instead"sv, a_death.formID);
				return WalkResult::kUnable;
			}
			// Mid-stride one foot is in the air: both stand on the lower one's ground, with the longer (straighter) leg. The
			// foot mid-swing at death steps first; a shot leg is knocked out from under the body: it starts off swinging
			// (planted, its hold soaked up the hit), and carries little weight (s.hurt).
			const float z = std::min(s.feet[0].offset.z, s.feet[1].offset.z), leg = std::max(s.feet[0].leg, s.feet[1].leg);
			const auto* hit = ref(a_death.hitBody);
			for (int i = 0; i < 2; ++i) {
				s.feet[i].offset.z = z;
				s.feet[i].leg = leg;
				const auto* r = ref(s.feet[i].id);
				const bool  shot = hit && hit->leg && hit->side && r && r->side == hit->side;
				if (shot) s.hurt = i;
				if (shot || (s.feet[i].id == a_death.swingAtDeath && s.swing < 0)) {
					s.swing = i;
					s.swingStart = s.time;
					s.swingTime = kWalkStepTime / (1.0f + 0.5f * Length(Flat(c.Vel(s.trunk[0].id))));  // as Stagger's steps
					s.swingFrom = c.Pos(s.feet[i].id);
					s.spot = s.swingFrom;
				}
			}
			REX::INFO("DynamicRagdollSystem: {:08X} death stagger: {} trunk bodies, {} arm bodies, pelvis {:.2f} up, {}{}"sv, a_death.formID,
				s.trunk.size(), s.arms.size(), s.standHeight, s.swing >= 0 ? "a foot mid-swing" : "both feet down", s.hurt >= 0 ? ", shot leg" : "");
		}
		// The pelvis and both legs must still be there (a severed leg ends it); trunk and arm bodies may go.
		const auto missing = [&](std::uint32_t a_id) { return !c.Body(a_id); };
		bool lost = missing(s.trunk[0].id);
		for (const auto& f : s.feet) lost |= missing(f.id) || missing(f.calf) || missing(f.thigh);
		if (lost) {
			REX::INFO("DynamicRagdollSystem: {:08X} death stagger: lost a leg after {} steps; collapses"sv, a_death.formID, a_death.steps);
			g_walkers.erase(a_death.formID);
			return WalkResult::kFallen;
		}
		// Ground: under the trunk, and under each foot where that ray is plausible (a missed one reads kRayDown below it).
		// Without rays everything stays on the ground at death (the flat fallback).
		if (a_death.groundValid) s.ground = a_death.groundTrunk;
		for (int i = 0; i < 2; ++i) {
			s.footGround[i] = s.ground;
			for (int k = 0; a_death.groundValid && k < a_death.groundFeet; ++k)
				if (a_death.groundFootId[k] == s.feet[i].id && std::abs(a_death.groundFoot[k] - s.ground) < kStepMaxRise) s.footGround[i] = a_death.groundFoot[k];
		}
		s.landing = a_death.stepGround;
		s.landingValid = a_death.stepGroundHit && s.swing >= 0 && a_death.stepFoot == s.feet[s.swing].id &&
		                 std::abs(a_death.stepGround - s.footGround[s.swing]) < kStepMaxRise;
		s.time += a_dt;

		// Everything fades steadily from the hit, so it loses the fight rather than being switched off: the steps get later,
		// slower and shorter, the legs sag, and the centre of pressure lags further behind the weight, so the ground tips
		// it on over its leading foot until it falls, still stepping. The trunk starts slack (the blow throws the upper body
		// and the legs chase it) and firms up over fDeathStaggerFlinch.
		const float strength = a_fade * a_help;
		const float legs = std::min(1.0f, 3.0f * a_fade) * a_help;
		const float flinch = 0.05f + 0.95f * std::clamp((a_now - a_death.start) / std::max(kDeathStaggerFlinch, 0.01f), 0.0f, 1.0f);
		// The tip comes in fully by a third of the way, while the legs still carry it; at once with a shot leg (it never could).
		const float tip = a_death.tip * (s.hurt >= 0 ? 1.0f : std::min(1.0f, 3.0f * (1.0f - a_fade)));
		c.stiffness = a_death.staggerUpright * flinch * (0.5f + 0.5f * a_fade);
		s.fallDir = a_death.staggerLine;  // it tips over the way the hit sent it
		const bool standing = Stagger(c, strength, legs, a_death.staggerBraking, tip);

		// Step bookkeeping for the landing ray (UpdateGround), the flail (leaves the stepping leg alone) and the log.
		const std::uint32_t swingId = s.swing >= 0 ? s.feet[s.swing].id : 0;
		if (!swingId) {
			a_death.stepFoot = 0;
			a_death.stepSide = 0;
		} else if (swingId != a_death.stepFoot && standing) {
			const auto* r = ref(swingId);
			a_death.stepFoot = swingId;
			a_death.stepSide = r ? r->side : 0;
			a_death.stepTo = s.spot;
			a_death.stepGroundTried = false;
			a_death.stepGroundHit = false;
			REX::INFO("DynamicRagdollSystem: {:08X} death stagger step {} with {} foot at {:.2f}s, length {:.2f}, strength {:.2f}"sv, a_death.formID,
				++a_death.steps, a_death.stepSide ? a_death.stepSide : '?', a_now - a_death.start, Length(Flat(Sub(s.spot, s.swingFrom))), strength);
		}
		return WalkResult::kWalking;
	}
}
