#pragma once

// The plugin's three death mechanics, one per killing shot (main.cpp: Q, W, E):
// - kWalk: the death walk (../src/Walk.cpp StepDeathWalk): the walker wanders drunkenly while its strength drains, then
//   collapses. Grip only.
// - kStagger: the death stagger (../src/DeathStagger.cpp): the walker staggering under the hit (Puppet::Stagger).
// - kStumble: the death stumble (../src/DeathStumble.cpp): the whole-rig balance controller (tilt, height, capture-point
//   steps, joint servo with knee buckle).
// Killing push, grip and brace are ported from ../src/Death.cpp (ApplyGrip, ApplyBrace) with its constants, except that
// the grip aims at the front of the wound.
// Left out: ground rays (the floor is flat, which is the plugin's no-ray fallback), weapon knockback trim, explosions,
// dismemberment, flailing while airborne, the roll odds (the key picks the mechanic), the walk turning into a stumble
// when shot again, and the dying phase after the stumble.

#include "Puppet.h"

#include <optional>
#include <random>

class Death
{
public:
	enum class Kind { kWalk, kStagger, kStumble };
	static const char* KindName(Kind a_kind);

	// The killing shot: captures the pose, rolls the death. a_pushScale: the plugin's per-hit pushScale (damage).
	void  Start(World& a_world, int a_hitPart, JPH::Vec3 a_hitDir, float a_pushScale = 1.0f, Kind a_kind = Kind::kStagger, bool a_grip = true);
	// Left click: which death the plugin would roll for a kill shot in a_hitPart (StartPendingDeaths, g_plugin's chances):
	// the walk (only headless if bDeathWalkHeadlessOnly, and the sim never dismembers), else the stagger, else the
	// stumble; a head shot may then skip it. None: a plain ragdoll fall. a_grip: rolled fDeathGripChance.
	static std::optional<Kind> Roll(World& a_world, int a_hitPart, bool& a_grip);
	void  Step(World& a_world, float a_dt);                           // before each physics step while Active()
	void  Stop() { m_active = false; }
	bool  Active() const { return m_active; }
	JPH::Vec3 Line() const { return m_line; }  // stagger direction (zero until the frame after the shot)
	float Left() const { return m_end - m_now; }
	bool  GivingWay() const { return m_active && m_now > 0.5f * m_end; }  // second half: the legs are failing

	static inline std::mt19937 rng{ std::random_device{}() };  // per-death rolls; the headless modes seed it so runs repeat

	Kind kind{ Kind::kStagger };
	int hitPart{ -1 }, gripHand{ -1 }, stepFoot{ -1 };  // debug view; -1 = none
	unsigned steps{};
	float    strength{}, legs{};  // what the stumble gives the walker this step (0 = limp)

private:
	struct Ref
	{
		bool pelvis{}, spine{}, leg{}, foot{}, hand{}, arm{}, knee{};
		char side{};
		int  priority{ 99 };
	};
	struct Sample
	{
		int       part;
		JPH::Vec3 pos, vel, angVel;
		JPH::Quat rot;
		float     mass;
	};

	struct Trunk
	{
		const Sample *low{}, *high{};  // pelvis, chest
		JPH::Vec3     up{}, forward{}, right{};
	};

	Trunk     TrunkFrame(World& a_world, const std::vector<Sample>& a_samples) const;
	JPH::Vec3 Route(const Trunk& a_trunk, const Sample& a_hand, JPH::Vec3 a_target, float* a_behind = nullptr) const;
	void      ApplyGrip(World& a_world, const std::vector<Sample>& a_samples, float a_dt, float a_strength);
	void ApplyBrace(World& a_world, const std::vector<Sample>& a_samples, float a_dt, float a_lean, float a_scale, float a_ground);
	float PushSpeed(World& a_world, int a_part) const;
	void  ApplyKillingPush(World& a_world, const std::vector<Sample>& a_samples, float a_scale);
	void  StepWalk(World& a_world, const std::vector<Sample>& a_samples, float a_dt);
	void  StepStumble(World& a_world, const std::vector<Sample>& a_samples, int a_trunk, JPH::Vec3 a_com, JPH::Vec3 a_comVel, float a_fade, float a_dt);

	std::vector<Ref>       m_refs;
	std::vector<JPH::Quat> m_targetRot;
	std::vector<JPH::Vec3> m_targetPos;
	Puppet       m_legs;  // the walker, staggering
	float        m_pushScale{ 1.0f };
	JPH::Vec3    m_hitDir{}, m_feetCaptured{}, m_braceFall{};
	JPH::Vec3    m_line{};  // stagger line: flat direction
	float        m_now{}, m_end{}, m_tip{}, m_pelvisHeight{ 1.0f }, m_upright{}, m_braking{}, m_braceLevel{};
	bool         m_active{}, m_hitApplied{}, m_gripChosen{}, m_grip{ true };
	// Death walk: wander direction (game heading), its next change, the zigzag and lurch.
	float m_heading{}, m_nextTurn{}, m_weavePhase{}, m_weaveRate{}, m_weaveSize{}, m_lurchPhase{};
	// Death stumble: rolled support (share of gravity carried); the step in progress; knee flexion axes (calf part, axis in
	// the thigh's frame).
	float     m_support{}, m_stepStart{}, m_stepTime{}, m_nextStep{};
	JPH::Vec3 m_stepFrom{}, m_stepTo{};
	char      m_lastStepSide{};
	std::vector<std::pair<int, JPH::Vec3>> m_kneeAxes;
};
