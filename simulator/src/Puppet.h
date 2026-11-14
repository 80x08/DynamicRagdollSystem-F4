#pragma once

// Port of src/Puppet.cpp's PuppetStep to Jolt. Same algorithm, same g_config.puppet values; the differences are only
// how bodies are read and pushed. Keep the two in step when changing the controller.

#include "World.h"

class Puppet
{
public:
	bool Capture(World& a_world, float a_yaw);                  // standing pose = the bodies right now; ground at z = 0
	void Step(World& a_world, JPH::Vec3 a_moveVel, float a_dt);  // before each physics step
	void KnockDown();                                            // key 1: limp now, then get up by itself
	// Death stumble (Death.cpp): captured from the pose right now (mid-stride too; a_swingPart = the foot in the air, or
	// -1), then Stagger each step instead of Step. a_strength scales every effort, a_legs is the share of the weight the
	// legs still carry, a_stiffness the trunk springs, a_brake (1/s) how hard it tries to stop, a_tip (m) how far the
	// centre of pressure lags behind the weight the way it is going (it tips over). False while no foot is down.
	bool BeginStagger(World& a_world, float a_yaw, int a_swingPart);
	bool Stagger(World& a_world, float a_strength, float a_legs, float a_stiffness, float a_brake, float a_tip, float a_dt);
	// Death walk (Death.cpp, Q): Walk toward a_moveVel with the drives scaled (strength: caps, support: lift, stiffness:
	// trunk springs) and the trunk turning at a_turnRate. False once fallen over (pelvis low or trunk tipped).
	bool DeathWalk(World& a_world, JPH::Vec3 a_moveVel, float a_strength, float a_support, float a_stiffness, float a_turnRate, float a_dt);
	int       hurt{ -1 };  // Stagger: the shot leg's foot (0/1): standing on it alone, it carries only fDeathStaggerHurtLeg of the weight
	JPH::Vec3 fallDir{};   // Stagger: flat direction the hit sent it (it tips over that way)

	bool weak{ false };  // key 4: soft trunk springs (fWeakSupport); tips past fGiveUpAngle -> falls and gets up

	enum class Rise { kNone, kDown, kRoll, kKneel, kLunge, kStand };  // getting up after a fall
	static const char* RiseName(Rise a_rise);

	// Debug view.
	JPH::Vec3 base{}, spot[2]{};
	int       swing{ -1 };
	float     yaw{};
	unsigned  steps{};
	Rise      rise{ Rise::kNone };
	unsigned  getUps{};  // finished get-ups
	int       FootPart(int a_i) const { return m_feet[a_i].part; }
	bool      IsTrunk(int a_part) const;

private:
	static constexpr float kContact = 0.05f;   // m above its standing height a foot still counts as on the ground
	static constexpr float kStraight = 0.97f;  // a leg counts as straight at this fraction of its standing length
	// shortcut: get-up timings and poses are constants; move them into g_config.puppet if they need tuning in game.
	static constexpr float kRollTime = 1.2f, kKneelTime = 1.2f, kLungeTime = 0.9f, kStandTime = 1.0f, kLungeReach = 0.55f;
	static constexpr float kBalanced = 0.1f, kToppled = 0.25f;  // key 4: m off the feet where lift starts fading / is gone
	static constexpr float kCatchStep = 1.0f;   // key 4: steps land this many times the weight's offset from the feet further along it
	static constexpr float kRegainTime = 3.0f;  // key 4: s after getting up before the trunk is fully soft again
	static constexpr float kAnkleAccel = 2.0f;  // key 4: m/s^2 sideways the feet can push the trunk back (~ g * half foot / hip height)
	static constexpr float kEaseIn = 0.5f, kRollLimbs = 0.2f, kArmKp = 4.0f;  // s; roll leg strength fraction; arm spring 1/s
	static constexpr float kLungeStart = 0.8f;  // fraction of kKneelTime at which the foot starts forward (the hips finish meanwhile)
	static constexpr float kLean = 0.25f;      // rad the chest leans over the front foot mid-rise
	static constexpr float kArmReach = 0.65f;   // m shoulder to planted hand before the hand lifts off
	static constexpr float kArmSpeed = 2.0f;    // m/s an arm drifts toward its pose, on top of the shoulder's own motion
	// Stagger (step length, slump and shot leg's share: g_plugin, [DeathStagger]): lowest pelvis (fraction of standing) a step's reach allows for; strength
	// below which it no longer steps; m the capture point may be off the feet before a step; m the centre of pressure may
	// be off the line between the feet (half a sole); ground friction; pelvis height (fraction of standing) once the legs
	// have given way; m the weight may be off the feet before the lift starts fading / is gone (as kBalanced, kToppled).
	static constexpr float kStaggerLow = 0.7f, kStaggerWeakest = 0.2f, kStaggerTrigger = 0.1f, kSole = 0.08f;
	static constexpr float kStaggerFriction = 0.6f, kBuckled = 0.4f, kStaggerBalanced = 0.2f, kStaggerToppled = 0.3f;
	static constexpr float kStaggerWide = 0.35f;  // m a foot may be from under its hip, flat, before it steps back in
	// m a shot leg's foot is lifted to fold its knee; pelvis height (fraction of standing)
	// below which, and m from under its hip beyond which, a leg left out folds the same way.
	static constexpr float kHurtLift = 0.12f, kStaggerSplit = 0.7f, kStaggerFar = 0.5f;
	struct TrunkPart { int part; JPH::Vec3 offset; JPH::Quat rot; };  // in the facing frame, from the pelvis
	struct Foot
	{
		int       part{ -1 }, calf{ -1 };
		JPH::Vec3 offset;     // x/y from the pelvis in the facing frame, z above the ground
		float     leg{};      // pelvis-to-foot length in the side view, m
		int       thigh{ -1 };
		float     thighLength{}, shinLength{}, thighCom{}, shinCom{};  // hip-knee, knee-ankle; centre of mass as a fraction of each
	};

	static constexpr float kFallenHeight = 0.55f, kFallenTilt = 0.5f;  // death walk: pelvis below this fraction of standing / trunk up axis z: fallen
	bool      Walk(World& a_world, JPH::Vec3 a_moveVel, float a_dt, float a_turnRate);
	void      Swing(World& a_world, JPH::Vec3 a_to, float a_time, JPH::Vec3 a_right, float a_dt);  // the swing foot along its arc
	JPH::Vec3 OwnSide(World& a_world, JPH::Vec3 a_p, int a_foot, JPH::Vec3 a_right) const;        // a_p kept clear of the other foot
	bool      GetUp(World& a_world, float a_dt);  // true while down or getting up (the walker then rests)
	JPH::Vec3 HoldTrunk(World& a_world, JPH::Vec3 a_base, JPH::Quat a_frame, JPH::Vec3 a_vel, float a_dt);  // returns the impulse given
	JPH::Vec3 DriveLeg(World& a_world, int a_foot, JPH::Vec3 a_ankle, JPH::Vec3 a_kneeAhead, float a_dt);   // returns the impulse given
	void      Brace(World& a_world, JPH::Vec3 a_reaction, const std::vector<int>& a_parts, float a_dt);     // the ground pushes back
	bool      OnGround(World& a_world, int a_part) const;
	JPH::Quat BodyFrame(World& a_world) const;  // the pelvis's current "facing frame" (standing: Facing(yaw))

	std::vector<TrunkPart> m_trunk;  // [0] = pelvis
	std::vector<int>       m_arms;
	Foot                   m_feet[2];
	int                    m_head{ -1 };  // highest trunk part
	float                  m_standHeight{ 1.0f }, m_ground{}, m_time{}, m_swingStart{}, m_landed{ -1.0e9f }, m_swingTime{};
	int                    m_lastSwing{ -1 };
	JPH::Vec3              m_root{}, m_swingFrom{};
	// Get-up state.
	float                  m_stageStart{}, m_riseYaw{}, m_rollTurn{};
	int                    m_front{ 0 };
	unsigned               m_lunges{};  // foot-forward attempts
	float                  m_strength{ 1.0f };  // scales every drive's acceleration cap (get-up stages ease in)
	float                  m_stiffness{ 1.0f };  // scales the trunk springs (key 4 walks soft)
	float                  m_upAt{ -1.0e9f };   // when it last got up
	float                  m_support{ 1.0f };    // scales the trunk's lift (key 4: fades as the weight leaves the feet)
	bool                   m_feetOnly{ true };   // a soft trunk is pushed sideways only as hard as the feet can (not in the death walk)
	JPH::Vec3              m_pivot{}, m_stageFrom{};
	JPH::Vec3              m_hands[2]{};  // where the hands push up from the floor while kneeling up (left, right)
};
