#include "Death.h"
#include "PluginSettings.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace JPH;

namespace
{
	// DynamicRagdollSystem's defaults (Common.h). Ranges are rolled per death.
	constexpr float kSwingingFoot = 1.0f;   // m/s faster than the other foot, flat, at death: that foot is mid-swing (it steps first)
	constexpr float kBraceLean = 0.35f, kBraceFull = 0.7f, kBraceGround = 0.2f, kBraceRampTime = 0.3f, kBraceTurnRate = 4.0f;
	constexpr float kGuardHandUp = 0.30f, kGuardHandForward = 0.22f, kGuardHandSide = 0.10f;
	constexpr float kGuardElbowUp = 0.02f, kGuardElbowForward = 0.32f, kGuardElbowSide = 0.17f;
	constexpr float kBraceKp = 120.0f, kBraceKd = 14.0f, kBraceMaxAccel = 300.0f;
	constexpr float kGripKp = 60.0f, kGripKd = 10.0f, kGripMaxAccel = 150.0f;
	// Hand behind the chest: m in front / out to the side of the wound it aims for (out also = how far from the pelvis's
	// centre it swings before turning forward), m over which each part of the detour ramps in, and m behind the chest's
	// centre before it starts (a hand that only lags a moment behind the shot chest goes straight).
	constexpr float kGripAround = 0.3f, kGripOut = 0.35f, kGripClear = 0.1f, kGripLag = 0.05f;
	constexpr float kGripUpperArmCarry = 0.3f;  // share of the upper arm's weight the grip carries while the hand is above the wound
	constexpr float kFloorBodyHeight = 0.36f;
	constexpr int   kPushRings = 3;
	constexpr float kGiveUpRange = 0.5f;
	// Pelvis height (fraction of standing) from which the stagger's drives fade as it goes down, and where it is down.
	constexpr float kFalling = 0.6f, kDown = 0.35f;
	// Stagger trunk stiffness (times the walker's springs), rolled per death (Common.h kStaggerUpright).
	constexpr float kDeathUpright[2]{ 0.8f, 1.0f };
	// Death stumble: the balance controller's constants (Common.h). Its tunable values are in g_plugin ([DeathStumble]).
	constexpr float kStumbleGiveUpTilt = 1.2f;
	constexpr float kJointKp = 20.0f, kJointKd = 4.0f, kJointMaxDeltaOmega = 3.0f, kJointMaxTorque = 250.0f;
	constexpr float kTiltKp = 25.0f, kTiltKd = 6.0f, kYawKd = 8.0f, kTiltMaxDeltaOmega = 1.5f, kHeightKp = 30.0f, kHeightKd = 6.0f;
	constexpr float kStepTrigger = 0.08f, kStepTime = 0.3f, kStepHeight = 0.2f, kStepMaxLength = 0.45f, kStepMinLength = 0.2f;
	constexpr float kSwingKneeBend = 50.0f, kKneeMinStiffness = 1.0f, kKneeMinFade = 0.4f, kStepCooldown = 0.12f;
	constexpr float kSwingKp = 20.0f, kSwingMaxAccel = 120.0f, kGroundTolerance = 0.2f, kBalanceMaxAccel = 40.0f;
	constexpr float kSupportBase = 0.4f, kSupportFade = 0.4f, kFootFriction = 0.8f, kSinkDepth = 0.25f;
	// Death walk (Walker.h): full strength for kWalkHold of the walk, trunk springs fade to kWalkSoft. The rest is in
	// g_plugin ([DeathWalk]).
	constexpr float kWalkHold = 0.4f, kWalkSoft = 0.35f;
	constexpr float kPi = 3.14159265f, kDegToRad = kPi / 180.0f;

	float Smooth(float a_t)
	{
		a_t = std::clamp(a_t, 0.0f, 1.0f);
		return a_t * a_t * (3.0f - 2.0f * a_t);
	}

	Vec3 Flat(Vec3 a_v) { return { a_v.GetX(), a_v.GetY(), 0.0f }; }
	Vec3 Unit(Vec3 a_v) { return a_v.NormalizedOr(Vec3::sZero()); }

	// World axis * angle that takes a_target to a_current (the plugin's RotationError).
	Vec3 RotationError(Quat a_current, Quat a_target)
	{
		Quat e = (a_current * a_target.Conjugated()).Normalized();
		if (e.GetW() < 0.0f) e = -e;
		Vec3  axis;
		float angle;
		e.GetAxisAngle(axis, angle);
		return angle < 1.0e-4f ? Vec3::sZero() : axis * angle;
	}
}

float Death::PushSpeed(World& a_world, int a_part) const
{
	// Joints from the hit body (JointsFromHit): breadth-first over the skeleton.
	std::vector<int> ring{ hitPart }, seen{ hitPart };
	const auto       has = [](const std::vector<int>& a_list, int a_v) { return std::find(a_list.begin(), a_list.end(), a_v) != a_list.end(); };
	for (int joints = 0; joints <= kPushRings && !ring.empty(); ++joints) {
		if (has(ring, a_part)) return joints == 0 ? g_plugin.hitPush : g_plugin.bodyPush * std::ldexp(1.0f, 1 - joints);
		std::vector<int> next;
		for (int i = 0; i < a_world.PartCount(); ++i) {
			const int parent = a_world.Part(i).parent;
			if (has(ring, parent) && !has(seen, i)) next.push_back(i), seen.push_back(i);
			if (has(ring, i) && parent >= 0 && !has(seen, parent)) next.push_back(parent), seen.push_back(parent);
		}
		ring.swap(next);
	}
	return 0.0f;
}

const char* Death::KindName(Kind a_kind)
{
	switch (a_kind) {
	case Kind::kWalk: return "DEATH WALK";
	case Kind::kStumble: return "DEATH STUMBLE";
	default: return "DEATH STAGGER";
	}
}

std::optional<Death::Kind> Death::Roll(World& a_world, int a_hitPart, bool& a_grip)
{
	const auto  chance = [](float a_p) { return std::uniform_real_distribution<float>{ 0.0f, 1.0f }(rng) < a_p; };
	const auto& p = g_plugin;
	const bool  headless = false;  // no dismemberment in the sim
	const bool  walk = (!p.walkHeadlessOnly || headless) && chance(p.walkChance);
	const bool  stagger = !walk && chance(p.staggerChance);
	const bool  stumble = stagger || (!walk && chance(p.stumbleChance));
	a_grip = chance(p.gripChance);
	const bool  headShot = a_world.Part(a_hitPart).name.find("Head") != std::string::npos;
	// Rolled first in priority: a skipped head shot neither walks, staggers nor stumbles (a headless-only walk never is).
	const bool  skipped = headShot && !(walk && p.walkHeadlessOnly) && (walk || stumble) && chance(p.headshotSkipChance);
	std::printf("rolled: walk %d stagger %d stumble %d grip %d%s\n", walk, stagger, stumble, a_grip, skipped ? ", head shot skipped it" : "");
	if (skipped || !(walk || stumble)) return std::nullopt;
	return walk ? Kind::kWalk : stagger ? Kind::kStagger : Kind::kStumble;
}

void Death::Start(World& a_world, int a_hitPart, Vec3 a_hitDir, float a_pushScale, Kind a_kind, bool a_grip)
{
	*this = {};
	kind = a_kind;
	m_grip = a_grip;
	m_pushScale = a_pushScale;
	auto&      bodies = a_world.Bodies();
	const auto pick = [&](const float (&a_range)[2]) { return std::uniform_real_distribution<float>{ a_range[0], a_range[1] }(rng); };
	hitPart = a_hitPart;
	m_hitDir = Unit(a_hitDir);
	m_end = pick(kind == Kind::kWalk ? g_plugin.walkTime : g_plugin.stumbleTime);
	m_tip = pick(g_plugin.staggerTip);
	m_upright = pick(kind == Kind::kStumble ? g_plugin.upright : kDeathUpright);
	m_braking = pick(kind == Kind::kStumble ? g_plugin.braking : g_plugin.staggerBraking);
	if (kind == Kind::kStumble) m_support = pick(g_plugin.support);  // only here: the seeded headless runs keep their rolls
	m_active = true;

	// Body flags as ScanBodies sets them from the bone names. Pose captured now, like the plugin's first frame.
	Vec3 feet = Vec3::sZero(), hips = Vec3::sZero();  // hips: flat, left to right across the thighs
	int  feetCount = 0;
	for (int i = 0; i < a_world.PartCount(); ++i) {
		const auto& name = a_world.Part(i).name;
		const auto  in = [&](const char* a_word) { return name.find(a_word) != std::string::npos; };
		Ref         r;
		r.pelvis = name == "COM" || in("Pelvis") || in("pelvis");
		r.spine = in("SPINE") || in("Spine") || in("Chest");
		r.leg = in("Leg") || in("Thigh") || in("Calf") || in("Foot");
		r.priority = r.pelvis ? -1 : in("SPINE1") ? 0 : in("SPINE2") ? 1 : in("Chest") ? 2 : 99;
		r.foot = in("Foot") || in("foot");
		r.side = name.starts_with("L") ? 'L' : name.starts_with("R") ? 'R' : 0;
		r.hand = in("Hand") || in("hand");
		r.arm = r.hand || in("Arm") || in("arm");
		r.knee = in("Calf");
		m_refs.push_back(r);
		m_targetRot.push_back(bodies.GetRotation(a_world.Body(i)));
		const Vec3 p(bodies.GetPosition(a_world.Body(i)));
		m_targetPos.push_back(p);
		if (r.foot) feet += p, ++feetCount;
		if (r.leg && !r.foot && !in("Calf") && r.side) hips += Flat(p) * (r.side == 'R' ? 1.0f : -1.0f);
	}
	m_feetCaptured = feetCount ? feet / static_cast<float>(feetCount) : Vec3::sZero();
	int swingAtDeath = -1;
	{  // the foot mid-swing at death, if any: clearly faster than the other (walking): it steps first
		int   fastest = -1;
		float speeds[2]{}, top = -1.0f;
		int   n = 0;
		for (int i = 0; i < a_world.PartCount() && n < 2; ++i)
			if (m_refs[i].foot) {
				speeds[n++] = Flat(bodies.GetLinearVelocity(a_world.Body(i))).Length();
				if (speeds[n - 1] > top) top = speeds[n - 1], fastest = i;
			}
		if (n == 2 && std::abs(speeds[0] - speeds[1]) > kSwingingFoot) swingAtDeath = fastest;
	}
	// A shot leg is knocked out from under the body: it starts off swinging (planted, its hold soaked up the hit).
	for (int i = 0; i < a_world.PartCount() && m_refs[a_hitPart].leg; ++i)
		if (m_refs[i].foot && m_refs[i].side == m_refs[a_hitPart].side) swingAtDeath = i;
	// Height: the lower foot, not the average. Dying mid-stride one foot is in the air.
	for (int i = 0; i < a_world.PartCount(); ++i)
		if (m_refs[i].foot) m_feetCaptured.SetZ(std::min(m_feetCaptured.GetZ(), float(bodies.GetPosition(a_world.Body(i)).GetZ())));
	const float facing = std::atan2(-hips.GetY(), hips.GetX());  // the way the hips face
	if (kind == Kind::kStagger) {
		// The legs and trunk are the walker's (key 4's catch steps).
		m_legs.BeginStagger(a_world, facing, swingAtDeath);
		for (int f = 0; f < 2; ++f)  // a shot leg: the walker's foot on that side
			if (m_refs[a_hitPart].leg && m_refs[m_legs.FootPart(f)].side == m_refs[a_hitPart].side) m_legs.hurt = f;
	} else if (kind == Kind::kWalk) {
		m_legs.Capture(a_world, facing);
		// Walking when killed: carries on that way, else the way it faces. The zigzag starts off to a random side.
		Vec3  moving = Vec3::sZero();
		float mass = 0.0f;
		for (int i = 0; i < a_world.PartCount(); ++i) {
			moving += Flat(bodies.GetLinearVelocity(a_world.Body(i))) * a_world.Part(i).mass;
			mass += a_world.Part(i).mass;
		}
		moving /= mass;
		m_heading = moving.Length() > 0.3f ? std::atan2(moving.GetX(), moving.GetY()) : facing;
		m_nextTurn = pick(g_plugin.walkDriftGap);
		m_weavePhase = std::uniform_real_distribution<float>{ 0.0f, 2.0f * kPi }(rng);
	}
	for (int i = 0; i < a_world.PartCount(); ++i)
		if (m_refs[i].pelvis) m_pelvisHeight = float(bodies.GetPosition(a_world.Body(i)).GetZ()) - m_feetCaptured.GetZ();
	std::printf("%s: hit %s, %.2f s, tip %.2f, upright %.2f, braking %.2f, support %.2f\n", KindName(kind), a_world.Part(a_hitPart).name.c_str(),
		m_end, m_tip, m_upright, m_braking, m_support);
}

// A hand clutches the body part the killing shot hit. Equal and opposite impulses on hand/arm and wound, so it acts
// like the arm's own muscles (no outside force). a_strength 0..1 lets the grip loosen.
void Death::ApplyGrip(World& a_world, const std::vector<Sample>& a_samples, float a_dt, float a_strength)
{
	if (!m_grip || hitPart < 0 || a_strength <= 0.0f) return;
	auto&         bodies = a_world.Bodies();
	const Sample& wound = a_samples[hitPart];
	if (!m_gripChosen) {
		m_gripChosen = true;
		float best = std::numeric_limits<float>::max();
		for (const auto& s : a_samples) {
			// An arm can't clutch its own shoulder or upper arm: a shot arm is clutched by the other hand.
			if (!m_refs[s.part].hand || s.part == hitPart || (m_refs[hitPart].arm && m_refs[s.part].side == m_refs[hitPart].side)) continue;
			if (const float d = (s.pos - wound.pos).Length(); d < best) {
				best = d;
				gripHand = s.part;
			}
		}
		std::printf("death: grip: %s clutches %s\n", gripHand >= 0 ? a_world.Part(gripHand).name.c_str() : "no hand", a_world.Part(hitPart).name.c_str());
	}
	if (gripHand < 0) return;
	const Sample& hand = a_samples[gripHand];
	// The hand alone is light: the joints spread its pull over the heavier forearm and upper arm and gravity wins. Size
	// the pull for the whole arm and carry the arm's weight while it clutches.
	const Vec3 g(0, 0, -g_sim.gravity);
	float      armMass = 0.0f;
	Vec3       woundImpulse = Vec3::sZero();
	for (const auto& s : a_samples) {
		if (!m_refs[s.part].arm || m_refs[s.part].side != m_refs[gripHand].side) continue;
		armMass += s.mass;
		// A hand above the wound: the upper arm only partly carried, so the elbow drops. Fully carried, a raised arm (aiming
		// at death) kept its elbow at shoulder height and the hand stuck under the chin. Below the wound (e.g. coming up
		// from behind the back to the head) the whole arm is carried, or it can't lift.
		const int  parent = a_world.Part(s.part).parent;
		const bool dropElbow = parent >= 0 && !m_refs[parent].arm && hand.pos.GetZ() > wound.pos.GetZ();
		const Vec3 lift = g * (-s.mass * a_dt * a_strength * (dropElbow ? kGripUpperArmCarry : 1.0f));
		bodies.AddImpulse(a_world.Body(s.part), lift);
		woundImpulse -= lift;
	}
	// The front of the wounded part, not its centre: pulled at the centre, a hand that reached the body high up pressed
	// straight into it and friction held it there (under the chin, on the collarbone).
	const Trunk trunk = TrunkFrame(a_world, a_samples);
	const Vec3  target = Route(trunk, hand, wound.pos + trunk.forward * a_world.Part(hitPart).radius);
	Vec3        deltaV = ((target - hand.pos) * kGripKp - (hand.vel - wound.vel) * kGripKd) * (a_dt * a_strength);
	const float n = deltaV.Length(), cap = kGripMaxAccel * a_dt;
	if (std::isfinite(n) && n > 1.0e-4f) {
		if (n > cap) deltaV *= cap / n;
		const Vec3 impulse = deltaV * std::max(armMass, hand.mass);
		bodies.AddImpulse(a_world.Body(gripHand), impulse);
		woundImpulse -= impulse;
	}
	// Equal and opposite. A shot arm: on the chest, which the clutching arm hangs from (on the shot arm, the other arm's
	// weight dragged it down).
	bodies.AddImpulse(a_world.Body(m_refs[hitPart].arm && trunk.high ? trunk.high->part : hitPart), woundImpulse);
}

// The trunk for steering the hands: pelvis and chest, and the chest's axes from the shoulders (no forward without them).
Death::Trunk Death::TrunkFrame(World& a_world, const std::vector<Sample>& a_samples) const
{
	Trunk         t;
	const Sample* shoulder[2]{};  // left and right upper arm
	for (const auto& s : a_samples) {
		const Ref& r = m_refs[s.part];
		if (r.spine || r.pelvis) {
			if (!t.low || r.priority < m_refs[t.low->part].priority) t.low = &s;
			if (!t.high || r.priority > m_refs[t.high->part].priority) t.high = &s;
		}
		const int parent = a_world.Part(s.part).parent;
		if (r.arm && r.side && parent >= 0 && !m_refs[parent].arm) shoulder[r.side == 'R'] = &s;
	}
	if (t.low && t.high && t.low != t.high && shoulder[0] && shoulder[1]) {
		t.up = Unit(t.high->pos - t.low->pos);
		t.forward = Unit(t.up.Cross(Unit(shoulder[1]->pos - shoulder[0]->pos)));
		t.right = t.forward.Cross(t.up);
	}
	return t;
}

// Where to pull a hand so it gets to a_target round the trunk. Pulled straight, a hand behind the trunk dragged the arm
// into the back and stuck there, and one reaching across the body (the other shoulder) pressed into the belly and stuck.
// Behind the chest: straight out to its own side past the hips, then round in front of the target. Across: out in front
// of the trunk first, then over. a_behind (optional): 0..1, how far the hand is behind the chest.
Vec3 Death::Route(const Trunk& a_trunk, const Sample& a_hand, Vec3 a_target, float* a_behind) const
{
	if (a_behind) *a_behind = 0.0f;
	if (a_trunk.forward.IsNearZero()) return a_target;
	const Vec3  forward = a_trunk.forward, out = a_trunk.right * (m_refs[a_hand.part].side == 'R' ? 1.0f : -1.0f);
	const Vec3  hand = a_hand.pos - a_trunk.high->pos;
	const float across = std::clamp(std::min(hand.Dot(out), -(a_target - a_trunk.high->pos).Dot(out)) / kGripClear, 0.0f, 1.0f);
	const float ahead = std::clamp((hand.Dot(forward) - kGripAround + kGripClear) / kGripClear, 0.0f, 1.0f);
	const Vec3  front = a_hand.pos + forward * kGripAround;  // a full kGripAround away, as sideways below
	const Vec3  target = a_target + (front + (a_target + forward * kGripAround - front) * ahead - a_target) * across;
	// Still within the body's width (behind the buttocks, the small of the back): straight out to the side first, or the
	// way round the front still runs up the back. A hand that only lags a moment behind the chest goes straight.
	const float detour = std::clamp((-hand.Dot(forward) - kGripLag) / kGripClear, 0.0f, 1.0f);
	const float aside = std::clamp(((a_hand.pos - a_trunk.low->pos).Dot(out) - kGripOut + kGripClear) / kGripClear, 0.0f, 1.0f);
	const Vec3  sideways = a_hand.pos + out * kGripOut;  // a full kGripOut away: a target near the hand pulls too weakly to free it
	const Vec3  around = a_target + forward * kGripAround + out * kGripOut;
	if (a_behind) *a_behind = detour;
	return target + (sideways + (around - sideways) * aside - target) * detour;
}

// Brace: when the body tips over, the free hands (not the one clutching the wound) curl up to guard the head in the
// fall direction. Until then a free hand behind the back is eased out of it. Equal and opposite impulses on hand/arm and
// chest, like the grip.
void Death::ApplyBrace(World& a_world, const std::vector<Sample>& a_samples, float a_dt, float a_lean, float a_scale, float a_ground)
{
	auto&       bodies = a_world.Bodies();
	const float wanted = std::clamp((a_lean - kBraceLean) / (kBraceFull - kBraceLean), 0.0f, 1.0f) * g_plugin.deathBrace * a_scale;
	const float ramp = g_plugin.deathBrace * a_dt / kBraceRampTime;  // eased in and out: switching on at full strength yanked the arms
	m_braceLevel += std::clamp(wanted - m_braceLevel, -ramp, ramp);
	const float amount = m_braceLevel;
	const Trunk trunk = TrunkFrame(a_world, a_samples);
	const Sample *low = trunk.low, *high = trunk.high;
	if (!low || !high || low == high) return;
	// A free hand thrown behind the back by the hit and the stagger came to rest there, the trunk leaning back over it, and
	// stayed, stiff. Eased out to its side and in front of the hips the way the grip goes round, and let go once out, so
	// the arm swings loose. Handed over to the brace as it comes in (which routes it the same way).
	Vec3 easeImpulse = Vec3::sZero();
	for (const auto& hand : a_samples) {
		if (!m_refs[hand.part].hand || hand.part == gripHand) continue;
		float      behind;
		const Vec3 target = Route(trunk, hand, low->pos, &behind);
		const float ease = behind * (1.0f - std::min(amount, 1.0f));
		if (ease <= 0.0f) continue;
		float armMass = 0.0f;
		for (const auto& s : a_samples)
			if (m_refs[s.part].arm && m_refs[s.part].side == m_refs[hand.part].side) armMass += s.mass;
		Vec3        deltaV = ((target - hand.pos) * kGripKp - (hand.vel - high->vel) * kGripKd) * (a_dt * ease);
		const float n = deltaV.Length(), cap = kGripMaxAccel * a_dt;
		if (!std::isfinite(n) || n < 1.0e-4f) continue;
		if (n > cap) deltaV *= cap / n;
		bodies.AddImpulse(a_world.Body(hand.part), deltaV * armMass);
		easeImpulse -= deltaV * armMass;
	}
	bodies.AddImpulse(a_world.Body(high->part), easeImpulse);
	if (amount <= 1.0e-4f) return;
	// Fall direction = where the trunk leans; nearly upright, where the chest is moving. Turned slowly (noisy input).
	Vec3 fall = Flat(high->pos - low->pos);
	if (fall.Length() < 0.05f) fall = Flat(high->vel);
	fall = fall.Length() < 1.0e-3f ? m_braceFall : fall.Normalized();
	if (m_braceFall.Length() > 0.5f) {
		const Vec3 eased = m_braceFall + (fall - m_braceFall) * std::min(1.0f, kBraceTurnRate * a_dt);
		fall = eased.Length() > 0.1f ? eased.Normalized() : m_braceFall;
	}
	if (fall.Length() < 0.5f) return;
	m_braceFall = fall;
	// Head guard in the trunk's frame (stays the same shape as the body tips).
	const Vec3 up = (high->pos - low->pos).Normalized();
	Vec3       forward = fall - up * fall.Dot(up);
	forward = forward.Length() > 0.05f ? forward.Normalized() : fall;
	// Falling backward: in front all the same (mirrored). Guarding the way it fell pulled the hands up behind the head,
	// crossed over (left and right flip with it).
	if (const float back = forward.Dot(trunk.forward); back < 0.0f) forward -= trunk.forward * (2.0f * back);
	const Vec3 right = forward.Cross(up);
	const auto guardPoint = [&](float a_up, float a_forward, float a_side) {
		Vec3 p = high->pos + up * a_up + forward * a_forward + right * a_side;
		p.SetZ(std::max(p.GetZ(), a_ground + 0.05f));
		return p;
	};
	const Vec3 g(0, 0, -g_sim.gravity);
	Vec3       chestImpulse = Vec3::sZero();
	for (const auto& hand : a_samples) {
		if (!m_refs[hand.part].hand || hand.part == gripHand) continue;
		const float   sideSign = m_refs[hand.part].side == 'L' ? -1.0f : 1.0f;
		float         behind;
		const Vec3    target = Route(trunk, hand, guardPoint(kGuardHandUp, kGuardHandForward, kGuardHandSide * sideSign), &behind);
		const int     parent = a_world.Part(hand.part).parent;
		const Sample* forearm = parent >= 0 && m_refs[parent].arm ? &a_samples[parent] : nullptr;
		// Carry the arm's weight while it reaches (otherwise it just sags), paid by the chest like a muscle.
		float armMass = 0.0f;
		for (const auto& s : a_samples) {
			if (!m_refs[s.part].arm || m_refs[s.part].side != m_refs[hand.part].side) continue;
			armMass += s.mass;
			const Vec3 lift = g * (-s.mass * a_dt * std::min(amount, 1.0f));
			bodies.AddImpulse(a_world.Body(s.part), lift);
			chestImpulse -= lift;
		}
		// Hand and forearm each pulled to their guard point, the arm's mass split between them.
		const auto drive = [&](const Sample& a_body, Vec3 a_target, float a_mass) {
			Vec3        deltaV = ((a_target - a_body.pos) * kBraceKp - (a_body.vel - high->vel) * kBraceKd) * (a_dt * amount);
			const float n = deltaV.Length(), cap = kBraceMaxAccel * a_dt;
			if (!std::isfinite(n) || n < 1.0e-4f) return;
			if (n > cap) deltaV *= cap / n;
			const Vec3 impulse = deltaV * std::max(a_mass, a_body.mass);
			bodies.AddImpulse(a_world.Body(a_body.part), impulse);
			chestImpulse -= impulse;
		};
		drive(hand, target, forearm ? 0.5f * armMass : armMass);
		if (forearm && behind <= 0.0f) drive(*forearm, guardPoint(kGuardElbowUp, kGuardElbowForward, kGuardElbowSide * sideSign), 0.5f * armMass);
	}
	bodies.AddImpulse(a_world.Body(high->part), chestImpulse);
}

// One-off push from the killing hit: the hit body, and bodies up to three joints away, halved per joint.
void Death::ApplyKillingPush(World& a_world, const std::vector<Sample>& a_samples, float a_scale)
{
	if (m_hitApplied) return;
	m_hitApplied = true;
	float lowest = std::numeric_limits<float>::max();
	for (const auto& s : a_samples) lowest = std::min(lowest, s.pos.GetZ());
	for (const auto& s : a_samples) {
		const float push = PushSpeed(a_world, s.part) * a_scale * m_pushScale;
		if (push <= 0.0f) continue;
		Vec3 impulse = m_hitDir * (push * s.mass);
		if (s.pos.GetZ() - lowest < kFloorBodyHeight) impulse.SetZ(std::max(impulse.GetZ(), 0.0f));  // floor-level bodies: not into the ground
		a_world.Bodies().AddImpulse(a_world.Body(s.part), impulse);
	}
}

void Death::Step(World& a_world, float a_dt)
{
	if (!m_active) return;
	auto&       bodies = a_world.Bodies();
	const float dt = a_dt;
	m_now += dt;
	// Strength decays steadily from the moment of death, so the body buckles progressively instead of standing at full
	// strength and dropping at the end. (The walk drains its own way, StepWalk, and ends here at the same time.)
	const float fade = std::clamp((m_end - m_now) / std::max(m_end, 0.01f), 0.0f, 1.0f);
	const float pelvisHeight = float(bodies.GetPosition(a_world.Body(0)).GetZ()) - m_feetCaptured.GetZ();
	// Stagger: going down, the drives fade with height, so it is limp once down, with nothing cut; or when time is up
	// (strength gone). The walk also ends when it falls (StepWalk), the stumble only when its time is up.
	const float up = kind == Kind::kStagger && m_hitApplied ? std::clamp((pelvisHeight / m_pelvisHeight - kDown) / (kFalling - kDown), 0.0f, 1.0f) : 1.0f;
	if (fade <= 0.0f || up <= 0.0f) {
		m_active = false;
		stepFoot = -1;
		std::printf("%s over after %u steps; limp\n", KindName(kind), steps);
		return;
	}

	// The body as it is now.
	std::vector<Sample> samples;
	Vec3                com = Vec3::sZero(), comVel = Vec3::sZero();
	float               mass = 0.0f;
	int                 trunk = -1;
	for (int i = 0; i < a_world.PartCount(); ++i) {
		const auto id = a_world.Body(i);
		samples.push_back({ i, Vec3(bodies.GetPosition(id)), bodies.GetLinearVelocity(id), bodies.GetAngularVelocity(id), bodies.GetRotation(id),
			a_world.Part(i).mass });
		const auto& s = samples.back();
		com += s.pos * s.mass;
		comVel += s.vel * s.mass;
		mass += s.mass;
		if (trunk < 0 || m_refs[i].priority < m_refs[trunk].priority) trunk = i;
	}
	com /= mass;
	comVel /= mass;
	if (kind == Kind::kWalk) return StepWalk(a_world, samples, dt);
	if (kind == Kind::kStumble) return StepStumble(a_world, samples, trunk, com, comVel, fade, dt);
	// Flat floor: the plugin's fallback when the ground rays miss (every level is the captured ground).
	const float rest = m_feetCaptured.GetZ(), groundSurface = rest - 0.1f;

	// Faster than a stumble, or leaning past g_plugin.staggerGiveUpTilt, cannot be caught: the help fades into a natural fall.
	Vec3 tiltError = RotationError(samples[trunk].rot, m_targetRot[trunk]);
	tiltError.SetZ(0.0f);
	const float speed = Flat(comVel).Length();
	const float lean = tiltError.Length();
	const float upright = std::clamp((g_plugin.staggerGiveUpTilt - lean) / kGiveUpRange, 0.0f, 1.0f);
	const float catchable = std::clamp(1.0f - (speed - g_plugin.staggerCatchSpeed) / g_plugin.staggerCatchRange, 0.0f, 1.0f) * upright;
	const float braceLean = com.GetZ() - rest > kBraceGround ? lean : 0.0f;

	// The stagger line (debug view): the way the body moves just after the shot (the shot's push plus whatever it was
	// already doing, e.g. walking).
	if (m_line.IsNearZero() && m_hitApplied) m_line = Flat(comVel).NormalizedOr(Flat(m_hitDir).NormalizedOr(Vec3::sAxisY()));
	m_legs.fallDir = m_line;  // it tips over the way the hit sent it (following its velocity, it fell back against it after a rebound)

	// 1. Legs and trunk: the walker staggering under the hit's momentum (Puppet::Stagger). Its help acts only through
	//    planted feet. Everything fades steadily from the hit, so it loses the fight rather than being switched off: the
	//    steps get later, slower and shorter, the legs sag, and the centre of pressure lags further behind the weight
	//    (m_tip), so the ground tips it on over its leading foot until it falls, still stepping. (A timed give-way
	//    folded it upright on the spot and it only toppled when it went limp.)
	strength = fade * catchable * up;
	legs = std::min(1.0f, 3.0f * fade) * catchable * up;
	const float flinch = 0.05f + 0.95f * std::clamp(m_now / g_plugin.staggerFlinch, 0.0f, 1.0f);
	// Full by a third of the way, while the legs still carry it; at once with a shot leg (it never could).
	const float tip = m_tip * (m_legs.hurt >= 0 ? 1.0f : std::min(1.0f, 3.0f * (1.0f - fade)));
	m_legs.Stagger(a_world, strength, legs, m_upright * flinch * (0.5f + 0.5f * fade), m_braking, tip, dt);
	const int swingFoot = m_legs.swing >= 0 ? m_legs.FootPart(m_legs.swing) : -1;
	if (swingFoot >= 0 && swingFoot != stepFoot)
		std::printf("death: step %u with %s at %.2f s\n", ++steps, a_world.Part(swingFoot).name.c_str(), m_now);
	stepFoot = swingFoot;

	ApplyKillingPush(a_world, samples, g_plugin.deathPushScale);

	// 2. Grip: a hand clutches the body part the killing shot hit; the free hands brace for the fall.
	ApplyGrip(a_world, samples, dt, 1.0f);
	ApplyBrace(a_world, samples, dt, braceLean, 1.0f, groundSurface);  // no flail in the sim, so full brace scale
}

// Death walk (../src/Walk.cpp StepDeathWalk): full strength for the first kWalkHold of the walk, then it drains to nothing;
// the trunk softens, sags and slows. The wander direction drifts every so often (sometimes a complete random turn), the
// path zigzags and the speed lurches. Over once it has fallen.
void Death::StepWalk(World& a_world, const std::vector<Sample>& a_samples, float a_dt)
{
	const auto  random = [&](float a_min, float a_max) { return std::uniform_real_distribution<float>{ a_min, a_max }(rng); };
	const float drain = 1.0f - Smooth((m_now / std::max(m_end, 0.01f) - kWalkHold) / (1.0f - kWalkHold));
	// Killing hit: the support starts knocked down and recovers, so the push shows before the walk catches it.
	strength = drain * (g_plugin.walkImpactSupport + (1.0f - g_plugin.walkImpactSupport) * Smooth(m_now / g_plugin.walkImpactTime));
	legs = strength;
	if (m_now >= m_nextTurn) {
		const float drift = g_plugin.walkDrift * kDegToRad;
		m_heading += random(0.0f, 1.0f) < g_plugin.walkSharpTurnChance ? random(-kPi, kPi) : random(-drift, drift);
		m_nextTurn = m_now + random(g_plugin.walkDriftGap[0], g_plugin.walkDriftGap[1]);
	}
	if (m_weaveRate <= 0.0f || m_weavePhase >= 2.0f * kPi) {
		m_weavePhase = std::fmod(m_weavePhase, 2.0f * kPi);
		m_weaveRate = 2.0f * kPi / std::max(random(g_plugin.walkWeaveTime[0], g_plugin.walkWeaveTime[1]), 0.1f);
		m_weaveSize = g_plugin.walkWeave * kDegToRad * random(0.4f, 1.0f);
		m_lurchPhase = random(0.0f, 2.0f * kPi);
	}
	m_weavePhase += m_weaveRate * a_dt;
	const float dir = m_heading + m_weaveSize * std::sin(m_weavePhase);
	const float lurch = std::max(1.0f + g_plugin.walkLurch * std::sin(2.0f * m_weavePhase + m_lurchPhase), 0.1f);
	const Vec3  moveVel = Vec3(std::sin(dir), std::cos(dir), 0.0f) * (g_plugin.walkSpeed * drain * lurch);
	ApplyKillingPush(a_world, a_samples, g_plugin.walkPushScale);  // the plugin pushes before the walk step
	if (!m_legs.DeathWalk(a_world, moveVel, std::max(strength, 0.05f), strength, kWalkSoft + (1.0f - kWalkSoft) * strength, g_plugin.walkTurnRate, a_dt)) {
		m_active = false;
		stepFoot = -1;
		std::printf("%s: fell after %.1f s, %u steps; limp\n", KindName(kind), m_now, steps);
		return;
	}
	const int swingFoot = m_legs.swing >= 0 ? m_legs.FootPart(m_legs.swing) : -1;
	if (swingFoot >= 0 && swingFoot != stepFoot) ++steps;
	stepFoot = swingFoot;
	ApplyGrip(a_world, a_samples, a_dt, 1.0f);
}

// Death stumble (../src/DeathStumble.cpp, the balance controller): every body gets the velocity change of one rigid body,
// so the whole mass is moved: tilted about the feet toward the captured trunk tilt (yaw only damped), the centre of mass
// sprung to its captured height (sinking as the knees buckle) with gravity partly cancelled, and only light braking
// sideways, so the hit's momentum turns into a stumble. Capture-point steps; all help handed to the planted feet; a joint
// servo holds the spine and legs to the death pose while the knees fold.
void Death::StepStumble(World& a_world, const std::vector<Sample>& a_samples, int a_trunk, Vec3 a_com, Vec3 a_comVel, float a_fade, float a_dt)
{
	auto&         bodies = a_world.Bodies();
	const float   dt = a_dt, fade = a_fade, gravity = g_sim.gravity;
	const Vec3    com = a_com, comVel = a_comVel;
	const float   rest = m_feetCaptured.GetZ(), groundSurface = rest - 0.1f;  // flat floor: every level is the captured ground
	const Sample& trunk = a_samples[a_trunk];
	Vec3          tiltError = RotationError(trunk.rot, m_targetRot[a_trunk]);
	tiltError.SetZ(0.0f);
	Vec3        omega = trunk.angVel;
	const float yawRate = omega.GetZ();
	omega.SetZ(0.0f);
	// Faster than g_plugin.catchSpeed, or leaning past kStumbleGiveUpTilt, cannot be caught: the help fades into a natural fall.
	const float speed = Flat(comVel).Length();
	const float lean = tiltError.Length();
	const float upright = std::clamp((kStumbleGiveUpTilt - lean) / kGiveUpRange, 0.0f, 1.0f);
	const float catchable = std::clamp(1.0f - (speed - g_plugin.catchSpeed) / g_plugin.catchRange, 0.0f, 1.0f) * upright;
	strength = fade * catchable;
	Vec3 dOmega = (tiltError * (-kTiltKp * m_upright) + omega * (-kTiltKd * m_upright)) * (dt * fade * catchable);
	if (const float n = dOmega.Length(); n > kTiltMaxDeltaOmega) dOmega *= kTiltMaxDeltaOmega / n;
	dOmega.SetZ(std::clamp(-kYawKd * yawRate * dt * fade * catchable, -kTiltMaxDeltaOmega, kTiltMaxDeltaOmega));

	const Sample* foot[2]{};
	int           footCount = 0;
	for (const auto& s : a_samples)
		if (m_refs[s.part].foot && footCount < 2) foot[footCount++] = &s;
	Vec3 feet = Vec3::sZero();
	for (int i = 0; i < footCount; ++i) feet += foot[i]->pos / static_cast<float>(footCount);
	float capturedZ = 0.0f, capturedMass = 0.0f;
	bool  grounded = false;  // any foot on the ground (two-sided: sunk through is not standing)
	for (const auto& s : a_samples) {
		capturedZ += m_targetPos[s.part].GetZ() * s.mass;
		capturedMass += s.mass;
		if (m_refs[s.part].foot && std::abs(s.pos.GetZ() - rest) < kGroundTolerance) grounded = true;
	}
	const float capturedComZ = capturedZ / capturedMass;
	const float braceLean = com.GetZ() - rest > kBraceGround ? lean : 0.0f;
	// The body sinks with the knee buckle instead of being held at standing height (which fought the bend).
	const float sink = kSinkDepth * std::min(1.0f, 1.5f * (1.0f - fade)) * std::clamp(g_plugin.kneeBend / 80.0f, 0.0f, 1.5f);
	// Support (gravity cancel + height spring) only while the feet are on the ground; airborne, gravity acts.
	const float support =
		grounded ? (kHeightKp * (capturedComZ - sink - com.GetZ()) - kHeightKd * comVel.GetZ() + gravity * m_support) * dt * fade * catchable : 0.0f;
	legs = grounded ? m_support * fade * catchable : 0.0f;
	const Vec3 dCom(-m_braking * comVel.GetX() * dt * catchable, -m_braking * comVel.GetY() * dt * catchable,
		grounded ? support : std::min(0.0f, -kHeightKd * comVel.GetZ() * dt));  // airborne: brake the launch only

	// Stepping: when the capture point (where the COM will come to rest) leaves the segment between the feet, swing a foot
	// under it. Feet alternate; the first step takes the one farther from the capture point.
	if (grounded && stepFoot < 0 && m_now >= m_nextStep && fade > 0.3f && catchable > 0.3f && footCount == 2) {
		const float look = std::sqrt(std::max(com.GetZ() - feet.GetZ(), 0.3f) / std::max(gravity, 1.0f));
		const Vec3  capture = Flat(com + comVel * look);
		const Vec3  a = Flat(foot[0]->pos), b = Flat(foot[1]->pos), ab = b - a;
		const float t = ab.LengthSq() > 1.0e-6f ? std::clamp((capture - a).Dot(ab) / ab.LengthSq(), 0.0f, 1.0f) : 0.5f;
		const Vec3  offset = capture - (a + ab * t);
		if (offset.Length() > kStepTrigger) {
			const auto flatDist = [&](const Sample* a_s) { return Flat(a_s->pos - capture).Length(); };
			bool       firstSwings = flatDist(foot[0]) > flatDist(foot[1]);
			if (m_lastStepSide && m_refs[foot[0]->part].side != m_refs[foot[1]->part].side) firstSwings = m_refs[foot[0]->part].side != m_lastStepSide;
			const Sample* swing = foot[firstSwings ? 0 : 1];
			const Sample* stance = foot[firstSwings ? 1 : 0];
			// Land under the capture point but keep half the stance width sideways.
			const Vec3 dir = offset.Normalized();
			Vec3       width = Flat(swing->pos - stance->pos);
			width -= dir * width.Dot(dir);
			Vec3 step = Flat(capture + width * 0.5f - swing->pos);
			// Faster bodies take longer strides so the feet keep up instead of the body sliding past them.
			const float maxStep = std::min(kStepMaxLength * (1.0f + 0.25f * speed), 2.0f * kStepMaxLength);
			if (const float n = step.Length(); n > maxStep) step *= maxStep / n;
			else if (n > 1.0e-3f && n < kStepMinLength) step *= kStepMinLength / n;
			stepFoot = swing->part;
			m_lastStepSide = m_refs[swing->part].side;
			m_stepFrom = swing->pos;
			m_stepTo = swing->pos + step;
			m_stepStart = m_now;
			m_stepTime = kStepTime / (1.0f + 0.3f * speed);  // quicker steps when moving fast
			std::printf("death: step %u with %s at %.2f s, length %.2f\n", ++steps, a_world.Part(swing->part).name.c_str(), m_now, step.Length());
		}
	}
	if (stepFoot >= 0) {
		const float phase = (m_now - m_stepStart) / m_stepTime;
		// Drop the step once the body can no longer catch itself (a full-strength swing kept yanking the foot).
		if (phase >= 1.0f || !grounded || catchable < 0.3f || fade < 0.15f) {
			stepFoot = -1;
			m_nextStep = m_now + kStepCooldown;
		} else {
			const Sample& swing = a_samples[stepFoot];
			const Vec3    desiredPos = m_stepFrom + (m_stepTo - m_stepFrom) * phase + Vec3(0.0f, 0.0f, kStepHeight * std::sin(kPi * phase));
			const Vec3    feedForward = (m_stepTo - m_stepFrom) / m_stepTime;
			Vec3          deltaV = feedForward + (desiredPos - swing.pos) * kSwingKp - swing.vel;
			const float   n = deltaV.Length(), cap = kSwingMaxAccel * dt;
			if (std::isfinite(n) && n > 1.0e-4f) {
				if (n > cap) deltaV *= cap / n;
				const Vec3 impulse = deltaV * swing.mass;
				bodies.AddImpulse(a_world.Body(stepFoot), impulse);
				// Hip-muscle reaction on the trunk: an unreacted swing push added momentum toward the capture point every
				// step, so the body sped up and slid past its planted leg.
				float trunkMass = 0.0f;
				for (const auto& s : a_samples)
					if (m_refs[s.part].pelvis || m_refs[s.part].spine) trunkMass += s.mass;
				for (const auto& s : a_samples)
					if (trunkMass > 0.0f && (m_refs[s.part].pelvis || m_refs[s.part].spine))
						bodies.AddLinearVelocity(a_world.Body(s.part), impulse * (-1.0f / trunkMass));
			}
		}
	}

	// All help acts through the planted feet: the opposite impulse goes into them, and sideways help is limited by foot
	// friction, so the body cannot hover or be held up against its momentum by an invisible hand. With its weight past the
	// feet (over a ledge) the help fades.
	std::vector<const Sample*> stance;
	for (int i = 0; i < footCount; ++i)
		if (foot[i]->part != stepFoot && std::abs(foot[i]->pos.GetZ() - rest) < kGroundTolerance) stance.push_back(foot[i]);
	float overFeet = 0.0f;
	if (!stance.empty()) {
		const Vec3  c = Flat(com), a = Flat(stance.front()->pos), b = Flat(stance.back()->pos), ab = b - a;
		const float t = ab.LengthSq() > 1.0e-6f ? std::clamp((c - a).Dot(ab) / ab.LengthSq(), 0.0f, 1.0f) : 0.0f;
		overFeet = std::clamp(1.0f - ((c - (a + ab * t)).Length() - kSupportBase) / kSupportFade, 0.0f, 1.0f);
	}
	struct Push
	{
		const Sample* s;
		Vec3          deltaV;
		float         turn;  // share of dOmega it turns by about its own centre
	};
	std::vector<Push> pushes;
	Vec3              total = Vec3::sZero();
	if (catchable > 0.0f && overFeet > 0.0f) {
		for (const auto& s : a_samples) {
			if (s.part == stepFoot || std::find(stance.begin(), stance.end(), &s) != stance.end()) continue;  // driven above / takes the reaction
			// Weight support goes to every body; the horizontal/tilt part skips the legs, which stay planted by friction.
			Vec3  deltaV(0.0f, 0.0f, dCom.GetZ());
			float turn = 0.0f;
			if (!m_refs[s.part].leg) {
				deltaV = dCom + dOmega.Cross(s.pos - feet);
				turn = m_refs[s.part].pelvis || m_refs[s.part].spine ? 1.0f : 0.0f;  // dOmega is the trunk's; loose limbs twitched
			}
			const float n = deltaV.Length(), cap = kBalanceMaxAccel * dt;
			if (!std::isfinite(n) || n < 1.0e-4f) continue;
			if (n > cap) {
				deltaV *= cap / n;
				turn *= cap / n;
			}
			pushes.push_back({ &s, deltaV * overFeet, turn * overFeet });
			total += deltaV * (overFeet * s.mass);
		}
	}
	// Friction cone: sideways help at most kFootFriction times the weight pressed into the ground.
	const float sideways = Flat(total).Length(), allowed = kFootFriction * std::max(total.GetZ(), 0.0f);
	const float sideScale = sideways > allowed && sideways > 1.0e-6f ? allowed / sideways : 1.0f;
	Vec3        reaction = Vec3::sZero();
	for (const auto& p : pushes) {
		const Vec3 impulse = Vec3(p.deltaV.GetX() * sideScale, p.deltaV.GetY() * sideScale, p.deltaV.GetZ()) * p.s->mass;
		reaction -= impulse;
		bodies.AddImpulse(a_world.Body(p.s->part), impulse);
		// A rigid rotation also turns the trunk bodies about their own centres (no reaction: small next to the arc).
		if (p.turn > 0.0f) bodies.AddAngularImpulse(a_world.Body(p.s->part), a_world.InvInertia(p.s->part).Inversed3x3().Multiply3x3(dOmega * p.turn));
	}
	reaction.SetZ(0.0f);  // sideways only: the ground supplies the upward part
	for (const auto* f : stance) bodies.AddImpulse(a_world.Body(f->part), reaction / static_cast<float>(stance.size()));
	ApplyKillingPush(a_world, a_samples, g_plugin.deathPushScale);

	// Grip: a hand clutches the body part the killing shot hit; the free hands brace for the fall.
	ApplyGrip(a_world, a_samples, dt, 1.0f);
	ApplyBrace(a_world, a_samples, dt, braceLean, 1.0f, groundSurface);  // no flail in the sim, so full brace scale

	// Knee flexion axes from the death pose: cross(thigh->calf, calf->foot) bends the knee further the way it already
	// bends; a straight leg borrows the other knee's axis. Kept in the thigh's frame.
	if (m_kneeAxes.empty()) {
		for (int i = 0; i < a_world.PartCount(); ++i) {
			const int thigh = a_world.Part(i).parent;
			if (!m_refs[i].knee || thigh < 0) continue;
			Vec3 axis = Vec3::sZero();
			for (int f = 0; f < a_world.PartCount(); ++f)
				if (m_refs[f].foot && m_refs[f].side == m_refs[i].side) {
					const Vec3 c = (m_targetPos[i] - m_targetPos[thigh]).Normalized().Cross((m_targetPos[f] - m_targetPos[i]).Normalized());
					if (c.Length() > 0.05f) axis = c.Normalized();
				}
			m_kneeAxes.push_back({ i, axis });
		}
		for (auto& [knee, axis] : m_kneeAxes)
			for (const auto& [other, otherAxis] : m_kneeAxes)
				if (axis.Length() < 0.5f && other != knee && otherAxis.Length() > 0.5f) axis = otherAxis;
		for (auto& [knee, axis] : m_kneeAxes) axis = m_targetRot[a_world.Part(knee).parent].Conjugated() * axis;
	}

	// Joint servo: paired internal torques (child +, parent -) restore the captured relative rotations. Arms are free
	// (grip, brace); the swing leg's hip is let go while its knee is driven to fold.
	for (int i = 0; i < a_world.PartCount(); ++i) {
		const int parent = a_world.Part(i).parent;
		if (parent < 0 || m_refs[i].arm) continue;
		const bool knee = m_refs[i].knee;
		const bool swingLeg = stepFoot >= 0 && m_refs[i].leg && m_refs[i].side == m_refs[stepFoot].side;
		if (swingLeg && !knee) continue;
		const Sample& child = a_samples[i];
		const Sample& par = a_samples[parent];
		const Quat    current = par.rot.Conjugated() * child.rot;
		Quat          target = m_targetRot[parent].Conjugated() * m_targetRot[i];
		if (knee) {
			for (const auto& [id, axis] : m_kneeAxes) {
				if (id != i || axis.Length() < 0.5f) continue;
				// Full bend by two thirds of the stumble; the stepping knee folds extra mid-swing.
				float bend = g_plugin.kneeBend * std::min(1.0f, 1.5f * (1.0f - fade));
				if (swingLeg) bend += kSwingKneeBend * std::sin(kPi * std::clamp((m_now - m_stepStart) / std::max(m_stepTime, 0.01f), 0.0f, 1.0f));
				target = Quat::sRotation(axis, bend * kDegToRad) * target;
			}
		}
		const Vec3  error = par.rot * RotationError(current, target);
		const Vec3  relative = child.angVel - par.angVel;
		const float stiffness = knee ? std::max(g_plugin.stiffness, kKneeMinStiffness) : g_plugin.stiffness;
		// Knees: a floor that ramps out over the last 10% instead of snapping off at the end.
		const float jointFade = knee ? std::max(fade, std::min(kKneeMinFade, 10.0f * fade)) : fade;
		Vec3        desired = (error * (-kJointKp * stiffness) + relative * (-kJointKd * stiffness)) * (dt * jointFade);
		const float n = desired.Length();
		if (!std::isfinite(n) || n < 0.002f) continue;
		if (n > kJointMaxDeltaOmega) desired *= kJointMaxDeltaOmega / n;
		// The impulse that changes the relative spin by `desired`: (Ic^-1 + Ip^-1)^-1 * desired, capped.
		const Mat44 inverse = a_world.InvInertia(i) + a_world.InvInertia(parent);
		if (std::abs(inverse.GetDeterminant3x3()) < 1.0e-12f) continue;
		Vec3        impulse = inverse.Inversed3x3().Multiply3x3(desired);
		const float size = impulse.Length(), cap = kJointMaxTorque * dt * jointFade;
		if (!std::isfinite(size) || size < 1.0e-6f) continue;
		if (size > cap) impulse *= cap / size;
		bodies.AddAngularImpulse(a_world.Body(i), impulse);
		bodies.AddAngularImpulse(a_world.Body(parent), -impulse);
	}
}
