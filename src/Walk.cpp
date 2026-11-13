#include "Common.h"

#include <numbers>
#include <unordered_map>

// Death walk: port of RagdollLab's active-ragdoll walker (RagdollLab src/Puppet.cpp: Walk, HoldTrunk, DriveLeg, Brace; its
// sim/src/Puppet.cpp is the same algorithm in Jolt). Keep the controller in step with RagdollLab.
// The dead body is the engine's limp death ragdoll; it is held up only through what touches the ground: the planted feet
// carry the trunk, the swing leg steps by two-bone IK. It wanders aimlessly while its strength drains (full for the first
// part of the walk, then fading to nothing), and collapses; the death's dying phase follows.

namespace DynamicRagdollSystem
{
	namespace
	{
		// RagdollLab's [Puppet] defaults (physics units = metres).
		constexpr float kWalkLeash = 0.35f;      // the walk target may run this far ahead of the real pelvis
		constexpr float kWalkCrouch = 0.9f;      // lowest the pelvis goes while walking, fraction of standing height
		constexpr float kWalkKp = 100.0f, kWalkKd = 18.0f, kWalkMaxAccel = 60.0f;      // trunk position spring
		constexpr float kWalkSpinKp = 60.0f, kWalkSpinKd = 14.0f, kWalkMaxSpin = 60.0f;  // trunk orientation spring
		constexpr float kWalkStepTime = 0.32f, kWalkStepLift = 0.15f, kWalkStepTrigger = 0.12f, kWalkStepLead = 0.5f;
		constexpr float kWalkSwingKp = 30.0f, kWalkSwingAccel = 150.0f, kWalkStanceHold = 60.0f;
		constexpr float kContact = 0.05f;   // a foot this far above its standing height still counts as on the ground
		constexpr float kStraight = 0.97f;  // a leg counts as straight at this fraction of its standing length
		// Wandering and dying (death walk only).
		constexpr float kWalkHold = 0.4f;                  // fraction of the walk at full strength; then it drains to nothing
		constexpr float kWalkSoft = 0.35f;                 // trunk springs at the end, fraction of full: it sways and sags
		constexpr float kFallenHeight = 0.55f;             // pelvis below this fraction of standing height: fallen
		constexpr float kFallenTilt = 0.5f;                // trunk's up axis below this (z): fallen
		constexpr float kGroundWait = 0.3f;                // s to wait for the ground ray before guessing it from the feet
		constexpr float kPi = std::numbers::pi_v<float>, kTwoPi = 2.0f * kPi, kDegToRad = kPi / 180.0f;

		constexpr Vec3 kX{ 1.0f, 0.0f, 0.0f }, kY{ 0.0f, 1.0f, 0.0f }, kZ{ 0.0f, 0.0f, 1.0f };

		struct Capsule { Vec3 a, b; float radius{}; };  // body-local segment

		struct Part
		{
			std::uint32_t id{};
			Vec3          offset;  // from the pelvis, in the facing frame
			Mat3          rot{};   // orientation in the facing frame
		};

		struct Foot
		{
			std::uint32_t id{}, calf{}, thigh{};
			Vec3          offset;  // x/y from the pelvis in the facing frame, z = height above the ground
			float         leg{};   // pelvis-to-foot length in the side view
			float         thighLength{}, shinLength{}, thighCom{}, shinCom{};  // hip-knee, knee-ankle; centre of mass as a fraction of each
		};

		struct Walker
		{
			float                                      deathStart{};  // the death this walker belongs to (a revived NPC dies again)
			std::vector<Part>                          trunk;         // trunk[0] = pelvis
			std::vector<std::uint32_t>                 arms;          // carried by the trunk's support
			Foot                                       feet[2];
			std::unordered_map<std::uint32_t, Capsule> shapes;
			float                                      standHeight{ 1.0f };  // pelvis above the ground
			float                                      yaw{};                // facing (game heading)
			float                                      heading{};            // wander direction (game heading)
			float                                      nextTurn{};
			float                                      weavePhase{}, weaveRate{}, weaveSize{}, lurchPhase{};  // drunken zigzag
			float                                      ground{};
			float                                      time{};               // simulated seconds (sum of Havok steps)
			Vec3                                       root;                 // where the pelvis should be (x/y)
			Vec3                                       moveVel;
			int                                        swing{ -1 }, lastSwing{ -1 };
			float                                      swingStart{}, landed{ -1.0e9f };
			Vec3                                       swingFrom;
			std::uint32_t                              steps{};
		};
		std::unordered_map<RE::TESFormID, Walker> g_walkers;  // main thread (Havok step, g_mutex held)

		Vec3 Flat(Vec3 a_v) { return { a_v.x, a_v.y, 0.0f }; }
		Vec3 Clamp(Vec3 a_v, float a_max)
		{
			const float n = Length(a_v);
			return n > a_max && n > 0.0f ? Scale(a_v, a_max / n) : a_v;
		}
		Vec3 UnitOr(Vec3 a_v, Vec3 a_fallback) { return Length(a_v) > 1.0e-6f ? Unit(a_v) : a_fallback; }
		float Smooth(float a_t)
		{
			a_t = std::clamp(a_t, 0.0f, 1.0f);
			return a_t * a_t * (3.0f - 2.0f * a_t);
		}
		float YawOf(Vec3 a_dir) { return std::atan2(a_dir.x, a_dir.y); }
		// Game heading (0 = +Y, clockwise) as a rotation about Z: forward (+Y local) -> (sin, cos), right (+X) -> (cos, -sin).
		Mat3 Yaw(float a_heading)
		{
			const float c = std::cos(a_heading), s = std::sin(a_heading);
			return { c, s, 0.0f, -s, c, 0.0f, 0.0f, 0.0f, 1.0f };
		}

		// hknpBody +0x48 shape; hknpCapsuleShape: convex radius +0x14, segment ends +0x50 / +0x60 (RE_NOTES, RagdollLab).
		bool ReadCapsule(const RE::hknpBody& a_body, Capsule& a_out)
		{
			std::uintptr_t shape = 0;
			float          a[4]{}, b[4]{};
			const auto     base = reinterpret_cast<std::uintptr_t>(&a_body);
			if (!SafeRead(base + 0x48, &shape, sizeof(shape)) || !shape || !SafeRead(shape + 0x14, &a_out.radius, sizeof(float)) ||
				!SafeRead(shape + 0x50, a, sizeof(a)) || !SafeRead(shape + 0x60, b, sizeof(b)))
				return false;
			a_out.a = { a[0], a[1], a[2] };
			a_out.b = { b[0], b[1], b[2] };
			return a_out.radius > 0.0f && a_out.radius < 0.5f && Length(Sub(a_out.a, a_out.b)) < 1.0f;
		}

		// One Havok step's view of the walker: reads and pushes bodies.
		struct Ctx
		{
			RE::hknpBSWorld*                                    world;
			Walker&                                             s;
			std::unordered_map<std::uint32_t, const RE::hknpBody*> bodies;  // this step's bodies by id
			float                                               dt, gravity;  // gravity: up is +
			float                                               strength{ 1.0f };   // scales every drive's acceleration cap
			float                                               stiffness{ 1.0f };  // scales the trunk springs and damping
			float                                               support{ 1.0f };    // scales the trunk's lift

			const RE::hknpBody* Body(std::uint32_t a_id) const
			{
				const auto it = bodies.find(a_id);
				return it == bodies.end() ? nullptr : it->second;
			}
			Vec3 Pos(std::uint32_t a_id) const
			{
				const auto* b = Body(a_id);
				return b ? BodyPosition(*b) : Vec3{};
			}
			Mat3 Rot(std::uint32_t a_id) const
			{
				const auto* b = Body(a_id);
				return b ? RotationFrom(b->m_transform.m_rotation) : Yaw(0.0f);
			}
			Vec3 World(std::uint32_t a_id, Vec3 a_local) const { return Add(Pos(a_id), Rotate(Rot(a_id), a_local)); }
			Vec3 Com(std::uint32_t a_id) const  // capsule middle
			{
				const auto it = s.shapes.find(a_id);
				return it == s.shapes.end() ? Pos(a_id) : World(a_id, Scale(Add(it->second.a, it->second.b), 0.5f));
			}
			bool Read(std::uint32_t a_id, MotionState& a_m) const
			{
				const auto* b = Body(a_id);
				return b && ReadMotionState(world, *b, a_m) && a_m.inverseMass > 0.0f;
			}
			Vec3 Vel(std::uint32_t a_id) const
			{
				MotionState m{};
				return Read(a_id, m) ? m.linear : Vec3{};
			}
			float Mass(std::uint32_t a_id) const
			{
				MotionState m{};
				return Read(a_id, m) ? 1.0f / m.inverseMass : 0.0f;
			}
			void Push(std::uint32_t a_id, Vec3 a_impulse) const
			{
				RE::hkVector4f impulse{ a_impulse.x, a_impulse.y, a_impulse.z, 0.0f };
				g_linearImpulse(world, a_id, &impulse);
			}
			void AddVelocity(std::uint32_t a_id, const MotionState& a_m, Vec3 a_dv) const { Push(a_id, Scale(a_dv, 1.0f / a_m.inverseMass)); }
			void AddVel(std::uint32_t a_id, Vec3 a_dv) const
			{
				MotionState m{};
				if (Read(a_id, m)) AddVelocity(a_id, m, a_dv);
			}
			void AddSpin(std::uint32_t a_id, const MotionState& a_m, Vec3 a_dw) const  // world-frame angular velocity change
			{
				float inverse[3][3]{};
				AddWorldInverse(a_m, inverse);
				Vec3 impulse{};
				if (!g_angularImpulse || !Solve3(inverse, a_dw, impulse)) return;
				RE::hkVector4f native{ impulse.x, impulse.y, impulse.z, 0.0f };
				g_angularImpulse(world, a_id, &native);
			}
			Mat3 BodyFrame() const { return Multiply(Rot(s.trunk[0].id), Transpose(s.trunk[0].rot)); }  // standing: Yaw(yaw)
		};

		// Trunk support: every trunk body is pulled toward its place in the standing pose, laid out in a_frame from a_base
		// (the pelvis), carrying its own weight plus the arms'. Returns the impulse, which the caller hands to the ground.
		Vec3 HoldTrunk(Ctx& c, Vec3 a_base, const Mat3& a_frame, Vec3 a_vel)
		{
			float trunkMass = 0.0f, armMass = 0.0f;
			for (const auto& t : c.s.trunk) trunkMass += c.Mass(t.id);
			for (const auto a : c.s.arms) armMass += c.Mass(a);
			Vec3 impulse{};
			if (trunkMass <= 0.0f) return impulse;
			const float carry = (trunkMass + armMass) / trunkMass;
			for (const auto& t : c.s.trunk) {
				MotionState m{};
				if (!c.Read(t.id, m)) continue;
				const auto target = Add(a_base, Rotate(a_frame, t.offset));
				auto dv = Scale(Add(Scale(Sub(target, c.Pos(t.id)), kWalkKp * c.stiffness), Scale(Sub(a_vel, m.linear), kWalkKd * c.stiffness)), c.dt);
				dv.z += c.gravity * c.dt * carry * c.support;
				dv = Clamp(dv, kWalkMaxAccel * c.strength * c.dt);
				c.AddVelocity(t.id, m, dv);
				impulse = Add(impulse, Scale(dv, 1.0f / m.inverseMass));
				// shortcut: the turning/upright torque is not pushed back into the ground (a capsule foot would just roll).
				const auto error = RotationError(c.Rot(t.id), Multiply(a_frame, t.rot));
				const auto angular = Rotate(m.orientation, m.angular);  // MotionState's angular velocity is body-local
				const auto dw = Scale(Sub(Scale(error, -kWalkSpinKp * c.stiffness), Scale(angular, kWalkSpinKd * c.stiffness)), c.dt);
				c.AddSpin(t.id, m, Clamp(dw, kWalkMaxSpin * c.strength * c.dt));
			}
			return impulse;
		}

		// Two-bone IK: where the knee goes for this ankle point (bent toward a_kneeAhead). The whole leg is driven to that one
		// shape, so the joints have nothing to fight; driving the foot alone just folds the knee. Returns the impulse given.
		Vec3 DriveLeg(Ctx& c, int a_foot, Vec3 a_ankle, Vec3 a_kneeAhead)
		{
			const auto& foot = c.s.feet[a_foot];
			const auto  hip = c.Pos(foot.thigh);  // bone origins: thigh = hip joint, calf = knee, foot = ankle
			auto        toAnkle = Sub(a_ankle, hip);
			const float reachMax = 0.999f * (foot.thighLength + foot.shinLength);
			if (Length(toAnkle) > reachMax) toAnkle = Scale(UnitOr(toAnkle, kZ), reachMax);
			const float d = std::max(Length(toAnkle), 1.0e-3f);
			const auto  u = Scale(toAnkle, 1.0f / d);
			const auto  n = UnitOr(Sub(a_kneeAhead, Scale(u, Dot(a_kneeAhead, u))), a_kneeAhead);
			const float along = (foot.thighLength * foot.thighLength - foot.shinLength * foot.shinLength + d * d) / (2.0f * d);
			const auto  knee = Add(Add(hip, Scale(u, along)), Scale(n, std::sqrt(std::max(0.0f, foot.thighLength * foot.thighLength - along * along))));
			const auto  ankle = Add(hip, toAnkle);
			const struct { std::uint32_t id; Vec3 target; } targets[]{
				{ foot.thigh, Add(hip, Scale(Sub(knee, hip), foot.thighCom)) },
				{ foot.calf, Add(knee, Scale(Sub(ankle, knee), foot.shinCom)) },
				{ foot.id, Add(ankle, Sub(c.Com(foot.id), c.Pos(foot.id))) },
			};
			Vec3 impulse{};
			for (const auto& [id, target] : targets) {
				MotionState m{};
				if (!c.Read(id, m)) continue;
				auto dv = Clamp(Sub(Scale(Sub(target, c.Com(id)), kWalkSwingKp), m.linear), kWalkSwingAccel * c.strength * c.dt);
				dv.z += c.gravity * c.dt;
				c.AddVelocity(id, m, dv);
				impulse = Add(impulse, Scale(dv, 1.0f / m.inverseMass));
			}
			return impulse;
		}

		// The supports take the reaction (split evenly) and hold still sideways: lift only exists through the ground.
		void Brace(Ctx& c, Vec3 a_reaction, const std::vector<std::uint32_t>& a_ids)
		{
			const float hold = std::min(1.0f, kWalkStanceHold * c.dt);
			for (const auto id : a_ids) {
				c.Push(id, Scale(a_reaction, -1.0f / static_cast<float>(a_ids.size())));
				c.AddVel(id, Scale(Flat(c.Vel(id)), -hold));
			}
		}

		// One step of walking toward s.moveVel (RagdollLab's Walk, firm mode). False while no foot is planted (limp).
		bool Walk(Ctx& c)
		{
			auto&       s = c.s;
			const float dt = c.dt;
			const auto  moveVel = s.moveVel;

			// Root follows the wander, leashed to the real pelvis; facing turns toward the walking direction.
			const auto pelvisPos = c.Pos(s.trunk[0].id);
			if (Length(moveVel) > 0.1f) {
				const float turn = std::remainder(YawOf(moveVel) - s.yaw, 2.0f * std::numbers::pi_v<float>);
				s.yaw += std::clamp(turn, -kDeathWalkTurnRate * dt, kDeathWalkTurnRate * dt);
			}
			const auto facing = Yaw(s.yaw);
			const auto right = Rotate(facing, kX);
			// Pelvis height rides over the planted legs like an inverted pendulum: as high as a nearly straight leg reaches
			// from each planted foot, never above standing and never below kWalkCrouch.
			const float lowest = s.standHeight * kWalkCrouch;
			const auto  legOf = [&](int a_foot) { return kStraight * s.feet[a_foot].leg; };
			const auto  reachAt = [&](int a_foot, float a_height) {
				const float leg = legOf(a_foot), drop = a_height - s.feet[a_foot].offset.z;
				return std::sqrt(std::max(0.0f, leg * leg - drop * drop));
			};
			bool down[2]{};
			for (int i = 0; i < 2; ++i) down[i] = c.Pos(s.feet[i].id).z - (s.ground + s.feet[i].offset.z) < kContact;
			const auto anchorOf = [&](int a_foot) { return Flat(Sub(c.Pos(s.feet[a_foot].id), Scale(right, s.feet[a_foot].offset.x))); };
			const auto lastRoot = s.root;
			s.root = Add(s.root, Scale(moveVel, dt));
			s.root = Add(Flat(pelvisPos), Clamp(Flat(Sub(s.root, pelvisPos)), kWalkLeash));
			// The body can't run ahead of its feet: the pelvis stays within a leg's reach of every planted foot.
			float height = s.standHeight;
			for (int i = 0; i < 2; ++i) {
				if (i == s.swing || !down[i]) continue;
				const auto anchor = anchorOf(i);
				s.root = Add(anchor, Clamp(Sub(Flat(s.root), anchor), reachAt(i, lowest)));
				const float d = Length(Sub(Flat(s.root), anchor)), leg = legOf(i);
				height = std::min(height, s.feet[i].offset.z + std::sqrt(std::max(0.0f, leg * leg - d * d)));
			}
			height = std::max(height, lowest);
			const auto rootVel = Clamp(Scale(Sub(s.root, lastRoot), 1.0f / dt), Length(moveVel));
			const Vec3 base{ s.root.x, s.root.y, s.ground + height };

			// 1. Stepping. Each foot's spot: under its hip, led ahead by the walking speed, and never across the other foot.
			const float side[2]{ s.feet[0].offset.x < s.feet[1].offset.x ? -1.0f : 1.0f, s.feet[0].offset.x < s.feet[1].offset.x ? 1.0f : -1.0f };
			const float minWidth = 0.6f * std::abs(s.feet[0].offset.x - s.feet[1].offset.x);
			const auto  ownSide = [&](Vec3 a_p, int a_foot) {
				const float lateral = Dot(Sub(a_p, c.Pos(s.feet[1 - a_foot].id)), right) * side[a_foot];
				return lateral < minWidth ? Add(a_p, Scale(right, side[a_foot] * (minWidth - lateral))) : a_p;
			};
			Vec3  spot[2];
			float off[2]{};
			for (int i = 0; i < 2; ++i) {
				const auto& o = s.feet[i].offset;
				spot[i] = Add(Add(base, Rotate(facing, { o.x, o.y, 0.0f })), Scale(moveVel, kWalkStepTime * kWalkStepLead));
				spot[i].z = s.ground + o.z;
				const auto hip = Add(Flat(pelvisPos), Scale(right, o.x));
				const auto reach = Clamp(Sub(Flat(spot[i]), hip), reachAt(i, lowest));
				spot[i] = ownSide({ hip.x + reach.x, hip.y + reach.y, spot[i].z }, i);
				off[i] = Length(Flat(Sub(spot[i], c.Pos(s.feet[i].id))));
			}
			// A swing ends when its time is up and the foot is down; a step starts only while the other foot stands.
			if (s.swing >= 0 && s.time - s.swingStart >= kWalkStepTime && down[s.swing]) {
				s.lastSwing = s.swing;
				s.landed = s.time;
				s.swing = -1;
			}
			// shortcut: fixed quarter-step of double support after each landing (as RagdollLab).
			if (s.swing < 0 && s.time - s.landed >= 0.25f * kWalkStepTime) {
				const int next = s.lastSwing >= 0 ? 1 - s.lastSwing : (off[0] > off[1] ? 0 : 1);  // feet alternate
				int       pick = off[next] > kWalkStepTrigger ? next : off[1 - next] > 1.5f * kWalkStepTrigger ? 1 - next : -1;
				if (pick >= 0 && !down[1 - pick]) pick = -1;
				if (pick >= 0) {
					s.swing = pick;
					s.swingStart = s.time;
					s.swingFrom = c.Pos(s.feet[pick].id);
					++s.steps;
				}
			}

			// 2. The planted feet carry the body: every impulse the trunk gets is pushed back into them.
			std::vector<std::uint32_t> planted;
			for (int i = 0; i < 2; ++i)
				if (i != s.swing && down[i]) planted.push_back(s.feet[i].id);
			if (planted.empty()) return false;  // airborne: limp until a foot comes down

			// 3. Swing foot: along a lifted arc to its spot; the leg's push is taken from the pelvis.
			if (s.swing >= 0) {
				const float phase = std::min(1.0f, (s.time - s.swingStart) / kWalkStepTime);
				const auto  to = spot[s.swing];
				const float travel = Smooth(phase);
				auto        arc = ownSide(Add(s.swingFrom, Scale(Sub(to, s.swingFrom), travel)), s.swing);
				arc.z = s.swingFrom.z + (to.z - s.swingFrom.z) * travel + kWalkStepLift * std::sin(std::numbers::pi_v<float> * std::min(1.0f, phase * 1.25f));
				c.Push(s.trunk[0].id, Scale(DriveLeg(c, s.swing, arc, Rotate(facing, kY)), -1.0f));
			}

			// 4. Support, handed to the planted feet; 5. their calves hold still sideways.
			Brace(c, HoldTrunk(c, base, facing, rootVel), planted);
			const float hold = std::min(1.0f, kWalkStanceHold * dt);
			for (int i = 0; i < 2; ++i)
				if (i != s.swing && down[i]) c.AddVel(s.feet[i].calf, Scale(Flat(c.Vel(s.feet[i].calf)), -hold));
			return true;
		}

		// Builds the walker from the death pose (a_death.targets: the animated pose captured on the first frame), like
		// RagdollLab's Puppet() from the live bodies. False if the skeleton lacks a pelvis, two feet, or a leg above each.
		bool Setup(const Death& a_death, RE::Actor* a_actor, const std::vector<const BodyRef*>& a_refs, const Ctx& a_c, Walker& a_s)
		{
			const auto target = [&](std::uint32_t a_id) -> const BodyTarget* {
				for (const auto& t : a_death.targets)
					if (t.id == a_id) return &t;
				return nullptr;
			};
			const auto ref = [&](std::uint32_t a_id) -> const BodyRef* {
				for (const auto* r : a_refs)
					if (r->id.m_value == a_id) return r;
				return nullptr;
			};
			const BodyRef* pelvis = nullptr;
			std::vector<const BodyRef*> feet;
			for (const auto* r : a_refs) {
				if (r->pelvis && !pelvis) pelvis = r;
				if (r->foot) feet.push_back(r);
				if (const auto* b = a_c.Body(r->id.m_value)) {
					Capsule capsule;
					if (ReadCapsule(*b, capsule)) a_s.shapes[r->id.m_value] = capsule;
				}
			}
			const auto* pt = pelvis ? target(pelvis->id.m_value) : nullptr;
			if (!pt || feet.size() != 2) return false;
			a_s.deathStart = a_death.start;
			a_s.yaw = a_actor->data.angle.z;
			const auto facing = Transpose(Yaw(a_s.yaw));
			const auto p = pt->pos;
			a_s.standHeight = std::clamp(p.z - a_s.ground, 0.5f, 1.5f);
			a_s.root = p;
			const auto comOf = [&](std::uint32_t a_id, const BodyTarget& a_t) {
				const auto it = a_s.shapes.find(a_id);
				return it == a_s.shapes.end() ? a_t.pos : Add(a_t.pos, Rotate(a_t.rot, Scale(Add(it->second.a, it->second.b), 0.5f)));
			};
			for (const auto* r : a_refs) {
				const auto* t = target(r->id.m_value);
				if (!t) continue;
				if ((r->pelvis || r->spine || r->head) && !r->leg && !r->arm) {
					const Part part{ r->id.m_value, Rotate(facing, Sub(t->pos, p)), Multiply(facing, t->rot) };
					if (r == pelvis) a_s.trunk.insert(a_s.trunk.begin(), part);
					else a_s.trunk.push_back(part);
				} else if (r->arm) {
					a_s.arms.push_back(r->id.m_value);
				}
			}
			for (int i = 0; i < 2; ++i) {
				const auto* calf = ref(feet[i]->parentId.m_value);
				const auto* thigh = calf ? ref(calf->parentId.m_value) : nullptr;
				const auto* ft = target(feet[i]->id.m_value);
				const auto* ct = calf ? target(calf->id.m_value) : nullptr;
				const auto* tt = thigh ? target(thigh->id.m_value) : nullptr;
				if (!ft || !ct || !tt) return false;
				const auto fp = ft->pos;
				auto       local = Rotate(facing, Sub(fp, p));
				local.z = fp.z - a_s.ground;
				auto& foot = a_s.feet[i];
				foot.id = feet[i]->id.m_value;
				foot.calf = calf->id.m_value;
				foot.thigh = thigh->id.m_value;
				foot.offset = local;
				foot.leg = std::sqrt(local.y * local.y + (fp.z - p.z) * (fp.z - p.z));
				// Leg bones for the IK: bone origins are the joints (hip, knee, ankle); centres of mass along each bone.
				const auto hip = tt->pos, knee = ct->pos;
				foot.thighLength = Length(Sub(knee, hip));
				foot.shinLength = Length(Sub(fp, knee));
				if (foot.thighLength < 0.05f || foot.shinLength < 0.05f) return false;
				foot.thighCom = Dot(Sub(comOf(foot.thigh, *tt), hip), Sub(knee, hip)) / (foot.thighLength * foot.thighLength);
				foot.shinCom = Dot(Sub(comOf(foot.calf, *ct), knee), Sub(fp, knee)) / (foot.shinLength * foot.shinLength);
			}
			return !a_s.trunk.empty();
		}
	}

	WalkResult StepDeathWalk(Death& a_death, RE::Actor* a_actor, RE::hknpBSWorld* a_world, float a_dt, float a_now)
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
		auto& s = c.s;
		if (s.deathStart != a_death.start || s.trunk.empty()) {
			// Ground: the ray at death; without one yet, wait a moment, then guess it from the lower foot.
			if (!a_death.groundCaptured && a_now - a_death.start < kGroundWait) return WalkResult::kWalking;
			s = Walker{};
			s.ground = a_death.groundCaptured ? a_death.groundAtDeath : a_death.feetCaptured.z - 0.1f;
			if (!Setup(a_death, a_actor, refs, c, s)) {
				g_walkers.erase(a_death.formID);
				REX::INFO("DynamicRagdollSystem: {:08X} death walk: skeleton not usable (needs a pelvis and two legs); stumbles instead"sv, a_death.formID);
				return WalkResult::kUnable;
			}
			const auto moving = Flat(a_death.moveVel);
			s.heading = Length(moving) > 0.3f ? YawOf(moving) : s.yaw;  // walking when killed: carries on that way
			s.nextTurn = Random(kDeathWalkDriftGap.min, kDeathWalkDriftGap.max);
			s.weavePhase = Random(0.0f, kTwoPi);  // starts off to a random side
			REX::INFO("DynamicRagdollSystem: {:08X} death walk: {} trunk bodies, {} arm bodies, {} capsules, pelvis {:.2f} up, legs {:.2f}/{:.2f}, for {:.1f}s"sv,
				a_death.formID, s.trunk.size(), s.arms.size(), s.shapes.size(), s.standHeight, s.feet[0].thighLength, s.feet[0].shinLength,
				a_death.end - a_death.start);
		}
		// Every body the walk needs must still be there (a severed leg ends it).
		const auto missing = [&](std::uint32_t a_id) { return !c.Body(a_id); };
		bool lost = std::any_of(s.trunk.begin(), s.trunk.end(), [&](const Part& a_p) { return missing(a_p.id); });
		for (const auto& f : s.feet) lost |= missing(f.id) || missing(f.calf) || missing(f.thigh);
		MotionState pelvis{};
		if (lost || !c.Read(s.trunk[0].id, pelvis)) {
			REX::INFO("DynamicRagdollSystem: {:08X} death walk: lost a body after {} steps; collapses"sv, a_death.formID, s.steps);
			g_walkers.erase(a_death.formID);
			return WalkResult::kFallen;
		}
		if (a_death.groundValid) s.ground = a_death.groundTrunk;
		s.time += a_dt;

		// Fallen over: let go (the death collapses into its dying phase).
		const float height = c.Pos(s.trunk[0].id).z - s.ground;
		if (height < kFallenHeight * s.standHeight || Rotate(c.BodyFrame(), kZ).z < kFallenTilt) {
			REX::INFO("DynamicRagdollSystem: {:08X} death walk: fell after {:.1f}s, {} steps"sv, a_death.formID, a_now - a_death.start, s.steps);
			g_walkers.erase(a_death.formID);
			return WalkResult::kFallen;
		}

		// Strength: full for the first part of the walk, then it drains to nothing; the trunk softens, sags and slows.
		const float span = std::max(a_death.end - a_death.start, 0.01f);
		const float progress = std::clamp((a_now - a_death.start) / span, 0.0f, 1.0f);
		const float drain = 1.0f - Smooth((progress - kWalkHold) / (1.0f - kWalkHold));
		// Killing hit: the support starts knocked down and recovers, so the push shows before the walk catches it.
		const float impact = kDeathWalkImpactTime > 0.0f ? Smooth((a_now - a_death.start) / kDeathWalkImpactTime) : 1.0f;
		const float strength = drain * (kDeathWalkImpactSupport + (1.0f - kDeathWalkImpactSupport) * impact);
		c.strength = std::max(strength, 0.05f);
		c.support = strength;
		c.stiffness = kWalkSoft + (1.0f - kWalkSoft) * strength;

		// Drunk: the wander direction drifts every so often, sometimes a complete random turn; on top of it the path
		// zigzags (each swing a new size and pace) and the speed lurches.
		if (s.time >= s.nextTurn) {
			const float drift = kDeathWalkDrift * kDegToRad;
			s.heading += Random() < kDeathWalkSharpTurnChance ? Random(-kPi, kPi) : Random(-drift, drift);
			s.nextTurn = s.time + Random(kDeathWalkDriftGap.min, kDeathWalkDriftGap.max);
		}
		if (s.weaveRate <= 0.0f || s.weavePhase >= kTwoPi) {
			s.weavePhase = std::fmod(s.weavePhase, kTwoPi);
			s.weaveRate = kTwoPi / std::max(Random(kDeathWalkWeaveTime.min, kDeathWalkWeaveTime.max), 0.1f);
			s.weaveSize = kDeathWalkWeave * kDegToRad * Random(0.4f, 1.0f);
			s.lurchPhase = Random(0.0f, kTwoPi);
		}
		s.weavePhase += s.weaveRate * a_dt;
		const float dir = s.heading + s.weaveSize * std::sin(s.weavePhase);
		const float lurch = std::max(1.0f + kDeathWalkLurch * std::sin(2.0f * s.weavePhase + s.lurchPhase), 0.1f);
		s.moveVel = Scale(Vec3{ std::sin(dir), std::cos(dir), 0.0f }, kDeathWalkSpeed * drain * lurch);
		Walk(c);
		return WalkResult::kWalking;
	}

	void EndDeathWalk(RE::TESFormID a_formID) { g_walkers.erase(a_formID); }

	void ClearDeathWalks() { g_walkers.clear(); }
}
