#pragma once

#include "Common.h"

#include <numbers>
#include <unordered_map>

// The active-ragdoll walker (RagdollLab Puppet) shared by the death walk (Walk.cpp) and the death stagger (DeathStagger.cpp).

namespace DynamicRagdollSystem::walker
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
	constexpr float kPi = std::numbers::pi_v<float>, kTwoPi = 2.0f * kPi, kDegToRad = kPi / 180.0f;
	// m/s^2 sideways the feet can push the trunk (HoldTrunk, stagger).
	constexpr float kAnkleAccel = 2.0f;

	constexpr Vec3 kX{ 1.0f, 0.0f, 0.0f }, kY{ 0.0f, 1.0f, 0.0f }, kZ{ 0.0f, 0.0f, 1.0f };

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
		// Stagger only.
		float                                      swingTime{};          // this step's length (s)
		int                                        hurt{ -1 };           // the shot leg's foot (0/1)
		Vec3                                       fallDir;              // flat: the way the hit sent it (it tips over that way)
		Vec3                                       spot;                 // where the stepping foot lands
		float                                      footGround[2]{};      // ground under each foot (rays, else ground)
		float                                      landing{};            // ground under the spot (step ray) ...
		bool                                       landingValid{};       // ... when it found one
	};
	inline std::unordered_map<RE::TESFormID, Walker> g_walkers;  // main thread (Havok step, g_mutex held)

	inline Vec3 Flat(Vec3 a_v) { return { a_v.x, a_v.y, 0.0f }; }
	inline Vec3 Clamp(Vec3 a_v, float a_max)
	{
		const float n = Length(a_v);
		return n > a_max && n > 0.0f ? Scale(a_v, a_max / n) : a_v;
	}
	inline Vec3 UnitOr(Vec3 a_v, Vec3 a_fallback) { return Length(a_v) > 1.0e-6f ? Unit(a_v) : a_fallback; }
	inline float Smooth(float a_t)
	{
		a_t = std::clamp(a_t, 0.0f, 1.0f);
		return a_t * a_t * (3.0f - 2.0f * a_t);
	}
	inline float YawOf(Vec3 a_dir) { return std::atan2(a_dir.x, a_dir.y); }
	// Game heading (0 = +Y, clockwise) as a rotation about Z: forward (+Y local) -> (sin, cos), right (+X) -> (cos, -sin).
	inline Mat3 Yaw(float a_heading)
	{
		const float c = std::cos(a_heading), s = std::sin(a_heading);
		return { c, s, 0.0f, -s, c, 0.0f, 0.0f, 0.0f, 1.0f };
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
		bool                                                feetOnly{ false };  // trunk pulled sideways only as hard as the feet can push (stagger)

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

	// Walk.cpp: the walker's drives, shared by the death walk and the death stagger (DeathStagger.cpp).
	// HoldTrunk: trunk support: every trunk body is pulled toward its place in the standing pose, laid out in a_frame from a_base
	// (the pelvis), carrying its own weight plus the arms'. Returns the impulse, which the caller hands to the ground.
	Vec3 HoldTrunk(Ctx& c, Vec3 a_base, const Mat3& a_frame, Vec3 a_vel);
	Vec3 DriveLeg(Ctx& c, int a_foot, Vec3 a_ankle, Vec3 a_kneeAhead);
	void Brace(Ctx& c, Vec3 a_reaction, const std::vector<std::uint32_t>& a_ids);
	void HoldStance(Ctx& c, const bool a_down[2]);
	Vec3 OwnSide(const Ctx& c, Vec3 a_p, int a_foot, Vec3 a_right);
	void Swing(Ctx& c, Vec3 a_to, float a_time, const Mat3& a_facing);
	bool Setup(const Death& a_death, float a_yaw, const std::vector<const BodyRef*>& a_refs, const Ctx& a_c, Walker& a_s);
}
