#include "Puppet.h"
#include "PluginSettings.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <numbers>

using namespace JPH;

namespace
{
	std::string Lower(std::string a_text)
	{
		std::transform(a_text.begin(), a_text.end(), a_text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		return a_text;
	}

	bool Has(const std::string& a_name, std::initializer_list<const char*> a_words)
	{
		return std::any_of(a_words.begin(), a_words.end(), [&](const char* w) { return a_name.find(w) != std::string::npos; });
	}

	// FO4 names every leg body "?Leg_*": "leg" alone also matches the feet and thighs (the nearest was the other foot).
	bool IsCalf(const std::string& a_name)
	{
		return Has(a_name, { "calf", "knee", "shin" }) || (Has(a_name, { "leg" }) && !Has(a_name, { "foot", "thigh", "toe" }));
	}

	Vec3 Clamp(Vec3 a_v, float a_max)
	{
		const float n = a_v.Length();
		return n > a_max && n > 0.0f ? a_v * (a_max / n) : a_v;
	}

	Vec3 Flat(Vec3 a_v) { return { a_v.GetX(), a_v.GetY(), 0.0f }; }

	// Game heading (0 = +Y, clockwise from above) as a rotation about Z, like the plugin's Yaw().
	Quat Facing(float a_yaw) { return Quat::sRotation(Vec3::sAxisZ(), -a_yaw); }

	// Facing a_yaw, tipped forward by a_angle about its own right axis (pi/2 = lying on the belly, head ahead).
	Quat Pitched(float a_yaw, float a_angle) { return Quat::sRotation(Facing(a_yaw) * Vec3::sAxisX(), -a_angle) * Facing(a_yaw); }

	float Smooth(float a_t)
	{
		a_t = std::clamp(a_t, 0.0f, 1.0f);
		return a_t * a_t * (3.0f - 2.0f * a_t);
	}

	// Starts and ends at rest like Smooth, but most of the way is covered early: an effort, then a settle.
	float Effort(float a_t) { return Smooth(std::pow(std::clamp(a_t, 0.0f, 1.0f), 0.85f)); }

	float YawOf(Vec3 a_dir) { return std::atan2(a_dir.GetX(), a_dir.GetY()); }

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

bool Puppet::IsTrunk(int a_part) const
{
	return std::any_of(m_trunk.begin(), m_trunk.end(), [&](const TrunkPart& t) { return t.part == a_part; });
}

bool Puppet::Capture(World& a_world, float a_yaw)
{
	*this = {};
	auto& bodies = a_world.Bodies();
	yaw = a_yaw;
	const Quat toFacing = Facing(yaw).Conjugated();

	int              pelvis = -1;
	std::vector<int> feet;
	for (int i = 0; i < a_world.PartCount(); ++i) {
		const auto name = Lower(a_world.Part(i).name);
		if ((name == "com" || Has(name, { "pelvis" })) && pelvis < 0) pelvis = i;
		if (Has(name, { "foot" })) feet.push_back(i);
	}
	if (pelvis < 0 || feet.size() != 2) {
		std::printf("Puppet refused: needs a pelvis and two feet\n");
		return false;
	}
	const Vec3 p = Vec3(bodies.GetPosition(a_world.Body(pelvis)));
	m_standHeight = std::clamp(p.GetZ() - m_ground, 0.5f, 1.5f);
	m_root = p;
	for (int i = 0; i < a_world.PartCount(); ++i) {
		const auto name = Lower(a_world.Part(i).name);
		const bool leg = Has(name, { "thigh", "calf", "foot", "leg", "toe", "knee", "ankle" });
		const auto id = a_world.Body(i);
		if ((i == pelvis || Has(name, { "pelvis", "spine", "chest", "neck", "head" })) && !leg) {
			TrunkPart part{ i, toFacing * (Vec3(bodies.GetPosition(id)) - p), toFacing * bodies.GetRotation(id) };
			if (i == pelvis) m_trunk.insert(m_trunk.begin(), part);
			else m_trunk.push_back(part);
		} else if (!leg) {
			m_arms.push_back(i);
		}
	}
	m_head = std::max_element(m_trunk.begin(), m_trunk.end(), [](const TrunkPart& a, const TrunkPart& b) {
		return a.offset.GetZ() < b.offset.GetZ();
	})->part;
	for (int f = 0; f < 2; ++f) {
		const Vec3 fp =Vec3(bodies.GetPosition(a_world.Body(feet[f])));
		Vec3       local = toFacing * (fp - p);
		local.SetZ(fp.GetZ() - m_ground);
		m_feet[f] = { feet[f], -1, local, std::sqrt(local.GetY() * local.GetY() + (fp.GetZ() - p.GetZ()) * (fp.GetZ() - p.GetZ())) };
		float best = 1.0e9f;  // calf: the nearest other leg body
		for (int i = 0; i < a_world.PartCount(); ++i) {
			const float d = (Vec3(bodies.GetPosition(a_world.Body(i))) - fp).Length();
			if (i != feet[f] && IsCalf(Lower(a_world.Part(i).name)) && d < best) {
				best = d;
				m_feet[f].calf = i;
			}
		}
		best = 1.0e9f;  // thigh: the nearest thigh to the calf
		for (int i = 0; i < a_world.PartCount() && m_feet[f].calf >= 0; ++i) {
			const float d = (Vec3(bodies.GetPosition(a_world.Body(i))) - Vec3(bodies.GetPosition(a_world.Body(m_feet[f].calf)))).Length();
			if (Has(Lower(a_world.Part(i).name), { "thigh" }) && d < best) {
				best = d;
				m_feet[f].thigh = i;
			}
		}
		auto& foot = m_feet[f];
		if (foot.calf < 0 || foot.thigh < 0) {
			std::printf("Puppet refused: each foot needs a calf and a thigh\n");
			return false;
		}
		// Leg bones for the swing's IK: bone origins are the joints (hip, knee, ankle); centres of mass along each bone.
		const auto origin = [&](int a_part) { return Vec3(bodies.GetPosition(a_world.Body(a_part))); };
		const auto com = [&](int a_part) { return Vec3(bodies.GetCenterOfMassPosition(a_world.Body(a_part))); };
		const Vec3 hip = origin(foot.thigh), knee = origin(foot.calf);
		foot.thighLength = (knee - hip).Length();
		foot.shinLength = (fp - knee).Length();
		foot.thighCom = (com(foot.thigh) - hip).Dot(knee - hip) / (foot.thighLength * foot.thighLength);
		foot.shinCom = (com(foot.calf) - knee).Dot(fp - knee) / (foot.shinLength * foot.shinLength);
	}
	std::printf("Puppet: %d trunk bodies, %d arm bodies, pelvis %.2f m above ground\n", static_cast<int>(m_trunk.size()),
		static_cast<int>(m_arms.size()), m_standHeight);
	return true;
}


const char* Puppet::RiseName(Rise a_rise)
{
	switch (a_rise) {
	case Rise::kDown: return "down";
	case Rise::kRoll: return "rolling over";
	case Rise::kKneel: return "kneeling up";
	case Rise::kLunge: return "foot forward";
	case Rise::kStand: return "standing up";
	default: return "";
	}
}

void Puppet::KnockDown()
{
	if (m_trunk.empty()) return;
	swing = -1;
	rise = Rise::kDown;  // GetUp waits for the body to lie still, then gets up
	m_stageStart = m_time;
}

void Puppet::Step(World& a_world, Vec3 a_moveVel, float a_dt)
{
	if (m_trunk.empty() || a_dt <= 0.0f) return;
	m_time += a_dt;
	++steps;
	m_strength = m_stiffness = m_support = 1.0f;
	if (!GetUp(a_world, a_dt)) {
		m_strength = 1.0f;
		// Springs and damping both: a soft trunk is loosely damped too. Just up from a fall it starts firm and softens.
		const float firm = Smooth((m_time - m_upAt) / kRegainTime);
		m_stiffness = weak ? 1.0f + (Lab::g_config.puppet.weakSupport - 1.0f) * firm : 1.0f;
		Walk(a_world, a_moveVel, a_dt, Lab::g_config.puppet.turnRate);
	}
}

bool Puppet::DeathWalk(World& a_world, Vec3 a_moveVel, float a_strength, float a_support, float a_stiffness, float a_turnRate, float a_dt)
{
	if (m_trunk.empty() || a_dt <= 0.0f) return false;
	const float height = float(a_world.Bodies().GetPosition(a_world.Body(m_trunk[0].part)).GetZ()) - m_ground;
	if (height < kFallenHeight * m_standHeight || (BodyFrame(a_world) * Vec3::sAxisZ()).GetZ() < kFallenTilt) return false;
	m_time += a_dt;
	m_strength = a_strength;
	m_support = a_support;
	m_stiffness = a_stiffness;
	m_feetOnly = false;  // the plugin's walk holds the trunk sideways at full spring (Ctx::feetOnly)
	Walk(a_world, a_moveVel, a_dt, a_turnRate);
	m_feetOnly = true;
	return true;
}

bool Puppet::OnGround(World& a_world, int a_part) const
{
	const auto& part = a_world.Part(a_part);
	const auto  t = a_world.Bodies().GetWorldTransform(a_world.Body(a_part));
	return std::min(Vec3(t * part.a).GetZ(), Vec3(t * part.b).GetZ()) - part.radius < m_ground + kContact;
}

Quat Puppet::BodyFrame(World& a_world) const
{
	return (a_world.Bodies().GetRotation(a_world.Body(m_trunk[0].part)) * m_trunk[0].rot.Conjugated()).Normalized();
}

// Trunk support: every trunk body is pulled toward its place in the standing pose, laid out in a_frame from a_base
// (the pelvis), carrying its own weight plus the arms'. Returns the impulse, which the caller hands to the ground.
Vec3 Puppet::HoldTrunk(World& a_world, Vec3 a_base, Quat a_frame, Vec3 a_vel, float a_dt)
{
	auto&       bodies = a_world.Bodies();
	const auto& cfg = Lab::g_config.puppet;
	float       trunkMass = 0.0f, armMass = 0.0f;
	for (const auto& t : m_trunk) trunkMass += a_world.Part(t.part).mass;
	for (const int a : m_arms) armMass += a_world.Part(a).mass;
	const float carry = (trunkMass + armMass) / trunkMass;
	Vec3        impulse = Vec3::sZero();
	for (const auto& t : m_trunk) {
		const auto id = a_world.Body(t.part);
		const Vec3 target = a_base + a_frame * t.offset;
		Vec3       dv = ((target - Vec3(bodies.GetPosition(id))) * (cfg.kp * m_stiffness) + (a_vel - bodies.GetLinearVelocity(id)) * (cfg.kd * m_stiffness)) * a_dt;
		if (m_feetOnly && m_stiffness < 1.0f) {  // key 4: sideways, only what pressing with the feet can do; the rest takes a step
			const Vec3 side = Clamp(Flat(dv), kAnkleAccel * a_dt);
			dv = Vec3(side.GetX(), side.GetY(), dv.GetZ());
		}
		dv.SetZ(dv.GetZ() + g_sim.gravity * a_dt * carry * m_support);
		dv = Clamp(dv, cfg.maxAccel * m_strength * a_dt);
		bodies.AddLinearVelocity(id, dv);
		impulse += dv * a_world.Part(t.part).mass;
		// shortcut: the turning/upright torque is not pushed back into the ground (a capsule foot would just roll); the
		// trunk keeps its orientation by itself. Route it through the supports as a force couple if that looks wrong.
		const Vec3 error = RotationError(bodies.GetRotation(id), a_frame * t.rot);
		const Vec3 spin = bodies.GetAngularVelocity(id);
		const Vec3 dw = (error * -(cfg.spinKp * m_stiffness) - spin * (cfg.spinKd * m_stiffness)) * a_dt;
		bodies.SetAngularVelocity(id, spin + Clamp(dw, cfg.maxSpin * m_strength * a_dt));  // game: angular impulse I*dw, same result
	}
	return impulse;
}

// Two-bone IK: where the knee goes for this ankle point (bent toward a_kneeAhead). The whole leg is driven to that one
// shape, so the joints have nothing to fight; driving the foot alone just folds the knee. Returns the impulse given.
Vec3 Puppet::DriveLeg(World& a_world, int a_foot, Vec3 a_ankle, Vec3 a_kneeAhead, float a_dt)
{
	auto&       bodies = a_world.Bodies();
	const auto& cfg = Lab::g_config.puppet;
	const auto& foot = m_feet[a_foot];
	const auto  pos = [&](int a_part) { return Vec3(bodies.GetPosition(a_world.Body(a_part))); };
	const auto  com = [&](int a_part) { return Vec3(bodies.GetCenterOfMassPosition(a_world.Body(a_part))); };
	const Vec3  hip = pos(foot.thigh);  // bone origins: thigh = hip joint, calf = knee, foot = ankle
	Vec3        toAnkle = a_ankle - hip;
	const float reachMax = 0.999f * (foot.thighLength + foot.shinLength);
	if (toAnkle.Length() > reachMax) toAnkle = toAnkle.Normalized() * reachMax;
	const float d = std::max(toAnkle.Length(), 1.0e-3f);
	const Vec3  u = toAnkle / d;
	const Vec3  n = (a_kneeAhead - u * a_kneeAhead.Dot(u)).NormalizedOr(a_kneeAhead);
	const float along = (foot.thighLength * foot.thighLength - foot.shinLength * foot.shinLength + d * d) / (2.0f * d);
	const Vec3  knee = hip + u * along + n * std::sqrt(std::max(0.0f, foot.thighLength * foot.thighLength - along * along));
	const Vec3  ankle = hip + toAnkle;
	const struct { int part; Vec3 target; } targets[]{
		{ foot.thigh, hip + (knee - hip) * foot.thighCom },
		{ foot.calf, knee + (ankle - knee) * foot.shinCom },
		{ foot.part, ankle + (com(foot.part) - pos(foot.part)) },
	};
	Vec3 impulse = Vec3::sZero();
	for (const auto& [part, target] : targets) {
		const auto id = a_world.Body(part);
		Vec3       dv = Clamp((target - com(part)) * cfg.swingKp - Vec3(bodies.GetLinearVelocity(id)), cfg.swingAccel * m_strength * a_dt);
		dv.SetZ(dv.GetZ() + g_sim.gravity * a_dt);
		bodies.AddLinearVelocity(id, dv);
		impulse += dv * a_world.Part(part).mass;
	}
	return impulse;
}

// The supports take the reaction (split evenly) and hold still sideways: lift only exists through the ground.
void Puppet::Brace(World& a_world, Vec3 a_reaction, const std::vector<int>& a_parts, float a_dt)
{
	auto&       bodies = a_world.Bodies();
	const float hold = std::min(1.0f, Lab::g_config.puppet.stanceHold * a_dt);
	for (const int part : a_parts) {
		const auto id = a_world.Body(part);
		bodies.AddImpulse(id, -a_reaction / static_cast<float>(a_parts.size()));
		bodies.AddLinearVelocity(id, -Flat(Vec3(bodies.GetLinearVelocity(id))) * hold);
	}
}

bool Puppet::Walk(World& a_world, Vec3 a_moveVel, float a_dt, float a_turnRate)
{
	auto&       bodies = a_world.Bodies();
	const auto& cfg = Lab::g_config.puppet;
	const auto  pos = [&](int a_part) { return Vec3(bodies.GetPosition(a_world.Body(a_part))); };

	// Root follows the input, leashed to the real pelvis; facing turns toward the walking direction.
	const Vec3 pelvisPos = pos(m_trunk[0].part);
	if (a_moveVel.Length() > 0.1f) {
		const float turn = std::remainder(YawOf(a_moveVel) - yaw, 2.0f * std::numbers::pi_v<float>);
		yaw += std::clamp(turn, -a_turnRate * a_dt, a_turnRate * a_dt);
	}
	const Quat facing = Facing(yaw);
	const Vec3 right = facing * Vec3::sAxisX();
	// Pelvis height rides over the planted legs like an inverted pendulum: as high as a nearly straight leg reaches from
	// each planted foot, never above standing and never below fCrouch. A fixed height can't walk: straight legs can't
	// reach forward, and bent ones use up the hip's swing range (FO4's hip flexes only ~45 degrees).
	const float lowest = m_standHeight * cfg.crouch;
	const auto  legOf = [&](int a_foot) { return kStraight * m_feet[a_foot].leg; };
	const auto  reachAt = [&](int a_foot, float a_height) {  // horizontal reach of a planted foot's hip at a pelvis height
		const float leg = legOf(a_foot), drop = a_height - m_feet[a_foot].offset.GetZ();
		return std::sqrt(std::max(0.0f, leg * leg - drop * drop));
	};
	bool down[2]{};
	for (int i = 0; i < 2; ++i) down[i] = pos(m_feet[i].part).GetZ() - (m_ground + m_feet[i].offset.GetZ()) < kContact;
	const auto anchorOf = [&](int a_foot) { return Flat(pos(m_feet[a_foot].part) - right * m_feet[a_foot].offset.GetX()); };
	const Vec3 lastRoot = m_root;
	m_root += a_moveVel * a_dt;
	m_root = Flat(pelvisPos) + Clamp(Flat(m_root - pelvisPos), cfg.leash);
	// The body can't run ahead of its feet: the pelvis stays within a leg's reach of every planted foot.
	float height = m_standHeight;
	for (int i = 0; i < 2; ++i) {
		if (i == swing || !down[i]) continue;
		const Vec3 anchor = anchorOf(i);
		m_root = anchor + Clamp(Flat(m_root) - anchor, reachAt(i, lowest));
		const float d = (Flat(m_root) - anchor).Length(), leg = legOf(i);
		height = std::min(height, m_feet[i].offset.GetZ() + std::sqrt(std::max(0.0f, leg * leg - d * d)));
	}
	height = std::max(height, lowest);
	const Vec3 rootVel = Clamp((m_root - lastRoot) / a_dt, a_moveVel.Length());  // what the trunk is asked to move at
	base = Vec3(m_root.GetX(), m_root.GetY(), m_ground + height);

	// 1. Stepping. Each foot's spot: under its hip, led ahead by the walking speed, and never across the other foot
	//    (turning would otherwise swing one leg through the other).
	// Key 4: the body's weight (trunk and arms) is not held over the feet, so the feet go to it: a push is caught by
	// stepping toward where the weight is heading. Too far off and it falls anyway (see the support below).
	Vec3 com = Vec3::sZero();
	if (weak) {
		float mass = 0.0f;
		for (const auto& t : m_trunk) {
			com += Vec3(bodies.GetCenterOfMassPosition(a_world.Body(t.part))) * a_world.Part(t.part).mass;
			mass += a_world.Part(t.part).mass;
		}
		for (const int a : m_arms) {
			com += Vec3(bodies.GetCenterOfMassPosition(a_world.Body(a))) * a_world.Part(a).mass;
			mass += a_world.Part(a).mass;
		}
		com = Flat(com / mass);
	}
	const Vec3 lean = weak ? com - 0.5f * (Flat(pos(m_feet[0].part)) + Flat(pos(m_feet[1].part))) : Vec3::sZero();
	float off[2]{};
	for (int i = 0; i < 2; ++i) {
		const Vec3& o = m_feet[i].offset;
		spot[i] = base + facing * Vec3(o.GetX(), o.GetY(), 0.0f) + a_moveVel * (cfg.stepTime * cfg.stepLead) + lean * kCatchStep;
		spot[i].SetZ(m_ground + o.GetZ());
		// No further from the hip than the leg reaches once the pelvis has come down to its lowest.
		const Vec3 hip = Flat(pelvisPos) + right * o.GetX();
		const Vec3 reach = Clamp(Flat(spot[i]) - hip, reachAt(i, lowest));
		spot[i] = OwnSide(a_world, Vec3(hip.GetX() + reach.GetX(), hip.GetY() + reach.GetY(), spot[i].GetZ()), i, right);
		off[i] = Flat(spot[i] - pos(m_feet[i].part)).Length();
	}
	// A swing ends when its time is up and the foot is down; a step starts only while the other foot stands.
	if (swing >= 0 && m_time - m_swingStart >= cfg.stepTime && down[swing]) {
		m_lastSwing = swing;
		m_landed = m_time;
		swing = -1;
	}
	// shortcut: fixed quarter-step of double support after each landing; make it a setting if it needs tuning.
	if (swing < 0 && m_time - m_landed >= 0.25f * cfg.stepTime) {
		const int next = m_lastSwing >= 0 ? 1 - m_lastSwing : (off[0] > off[1] ? 0 : 1);  // feet alternate
		int       pick = off[next] > cfg.stepTrigger ? next : off[1 - next] > 1.5f * cfg.stepTrigger ? 1 - next : -1;
		if (pick < 0 && lean.Length() > kBalanced) pick = next;  // key 4: weight drifting off the feet: step under it
		if (pick >= 0 && !down[1 - pick]) pick = -1;
		if (pick >= 0) {
			swing = pick;
			m_swingStart = m_time;
			m_swingFrom = pos(m_feet[pick].part);
		}
	}

	// 2. The planted feet carry the body. Every impulse the controller gives the trunk is pushed back into them, so lift
	//    and drive only exist through the ground: the foot is pressed down with the body's weight (friction holds it),
	//    and with no foot down there is no support at all.
	std::vector<int> planted;
	for (int i = 0; i < 2; ++i)
		if (i != swing && down[i]) planted.push_back(m_feet[i].part);
	if (planted.empty()) return false;  // airborne: limp until a foot comes down

	// 3. Swing foot: along a lifted arc to its spot, on its own side of the planted foot. The hip swings the leg against
	//    the pelvis (the leg's push is taken from the pelvis, not the ground), so swinging moves the leg, not the body;
	//    the support below then hands the pelvis's share on to the planted feet.
	if (swing >= 0) Swing(a_world, spot[swing], cfg.stepTime, right, a_dt);

	// 4. Support: the trunk carries itself and the arms (the legs stand on the ground themselves); 5. the planted feet
	//    take it, holding still sideways with their calves.
	// Key 4: the legs only push up while the body's weight is over them; off balance the lift fades and it falls. Mid-step
	// "over them" means between the standing foot and where the stepping foot lands: a step is how a fall is caught.
	if (weak) {
		const Vec3 a = Flat(Vec3(bodies.GetPosition(a_world.Body(planted.front()))));
		const Vec3 b = swing >= 0 ? Flat(pos(m_feet[swing].part)) : Flat(Vec3(bodies.GetPosition(a_world.Body(planted.back()))));
		const Vec3 ab = b - a;
		const float along = ab.LengthSq() > 1.0e-6f ? std::clamp((com - a).Dot(ab) / ab.LengthSq(), 0.0f, 1.0f) : 0.0f;
		m_support = 1.0f - Smooth(((com - (a + ab * along)).Length() - kBalanced) / kToppled);  // distance off the feet
	}
	Brace(a_world, HoldTrunk(a_world, base, facing, rootVel, a_dt), planted, a_dt);
	const float hold = std::min(1.0f, cfg.stanceHold * a_dt);
	for (int i = 0; i < 2; ++i) {
		if (i == swing || !down[i]) continue;
		const auto calf = a_world.Body(m_feet[i].calf);
		bodies.AddLinearVelocity(calf, -Flat(Vec3(bodies.GetLinearVelocity(calf))) * hold);
	}
	return true;
}

Vec3 Puppet::OwnSide(World& a_world, Vec3 a_p, int a_foot, Vec3 a_right) const
{
	const float side = m_feet[a_foot].offset.GetX() < m_feet[1 - a_foot].offset.GetX() ? -1.0f : 1.0f;
	const float minWidth = 0.6f * std::abs(m_feet[0].offset.GetX() - m_feet[1].offset.GetX());
	const float lateral = (a_p - Vec3(a_world.Bodies().GetPosition(a_world.Body(m_feet[1 - a_foot].part)))).Dot(a_right) * side;
	return lateral < minWidth ? a_p + a_right * (side * (minWidth - lateral)) : a_p;
}

void Puppet::Swing(World& a_world, Vec3 a_to, float a_time, Vec3 a_right, float a_dt)
{
	const float phase = std::min(1.0f, (m_time - m_swingStart) / a_time);  // past 1: set down on the spot
	// Lift first, then reach: the foot rises (bending the knee) before it travels, peaks at 40% and is down by 80%, so
	// a straight leg never has to sweep along the floor under the hip.
	const float travel = Smooth(phase);
	Vec3        arc = OwnSide(a_world, m_swingFrom + (a_to - m_swingFrom) * travel, swing, a_right);
	arc.SetZ(m_swingFrom.GetZ() + (a_to.GetZ() - m_swingFrom.GetZ()) * travel +
	         Lab::g_config.puppet.stepLift * std::sin(std::numbers::pi_v<float> * std::min(1.0f, phase * 1.25f)));
	a_world.Bodies().AddImpulse(a_world.Body(m_trunk[0].part), -DriveLeg(a_world, swing, arc, Vec3::sAxisZ().Cross(a_right), a_dt));
}

bool Puppet::BeginStagger(World& a_world, float a_yaw, int a_swingPart)
{
	if (!Capture(a_world, a_yaw)) return false;
	// Mid-stride one foot is in the air: both stand on the lower one's ground, with the longer (straighter) leg.
	const float z = std::min(m_feet[0].offset.GetZ(), m_feet[1].offset.GetZ()), leg = std::max(m_feet[0].leg, m_feet[1].leg);
	for (int i = 0; i < 2; ++i) {
		m_feet[i].offset.SetZ(z);
		m_feet[i].leg = leg;
		if (m_feet[i].part == a_swingPart) {
			swing = i;
			m_swingStart = m_time;
			const float speed = Flat(Vec3(a_world.Bodies().GetLinearVelocity(a_world.Body(m_trunk[0].part)))).Length();
			m_swingTime = Lab::g_config.puppet.stepTime / (1.0f + 0.5f * speed);  // as Stagger's steps
			m_swingFrom = Vec3(a_world.Bodies().GetPosition(a_world.Body(a_swingPart)));
		}
	}
	return true;
}

// The weak walker with no goal of its own: the hit's momentum carries the body and the feet step to catch it. The
// trunk is held up and upright through the planted feet as in Walk, but pushed sideways only as hard as legs standing
// there can push: like an inverted pendulum, the ground force points from the centre of pressure (somewhere under the
// planted feet) to the weight, so it can brake the body only while the feet are ahead of it. Each step therefore lands
// where the weight is heading (its capture point, as it will be at touchdown), which is what slows it down.
bool Puppet::Stagger(World& a_world, float a_strength, float a_legs, float a_stiffness, float a_brake, float a_tip, float a_dt)
{
	auto&       bodies = a_world.Bodies();
	const auto& cfg = Lab::g_config.puppet;
	const auto  pos = [&](int a_part) { return Vec3(bodies.GetPosition(a_world.Body(a_part))); };
	if (m_trunk.empty() || a_dt <= 0.0f) return false;
	m_time += a_dt;
	m_strength = a_strength;
	m_stiffness = a_stiffness;
	const Vec3 ahead = Flat(BodyFrame(a_world) * Vec3::sAxisY());  // the trunk faces where the pelvis now faces
	if (ahead.Length() > 0.1f) yaw = YawOf(ahead);
	const Quat facing = Facing(yaw);
	const Vec3 right = facing * Vec3::sAxisX();

	// The weight (trunk and arms; the legs stand on the ground themselves) and its capture point.
	Vec3  com = Vec3::sZero(), vel = Vec3::sZero();
	float mass = 0.0f, trunkMass = 0.0f;
	for (const auto& t : m_trunk) trunkMass += a_world.Part(t.part).mass;
	for (int i = 0; i < a_world.PartCount(); ++i) {
		if (!IsTrunk(i) && std::find(m_arms.begin(), m_arms.end(), i) == m_arms.end()) continue;
		com += Vec3(bodies.GetCenterOfMassPosition(a_world.Body(i))) * a_world.Part(i).mass;
		vel += Vec3(bodies.GetLinearVelocity(a_world.Body(i))) * a_world.Part(i).mass;
		mass += a_world.Part(i).mass;
	}
	com /= mass;
	vel /= mass;
	const float g = g_sim.gravity, height = std::max(com.GetZ() - m_ground, 0.3f), omega = std::sqrt(g / height);
	const Vec3  capture = Flat(com) + Flat(vel) / omega;
	const float speed = Flat(vel).Length();

	bool down[2]{};
	Vec3 foot[2]{};
	for (int i = 0; i < 2; ++i) {
		foot[i] = pos(m_feet[i].part);
		down[i] = foot[i].GetZ() - (m_ground + m_feet[i].offset.GetZ()) < kContact;
		foot[i] = Flat(foot[i]);
	}
	// A step is over once the foot is down and its time is up, or once the other foot has left the ground (rolled off
	// its toes): the weight is on it now.
	if (swing >= 0 && down[swing] && (m_time - m_swingStart >= m_swingTime || !down[1 - swing])) {
		m_lastSwing = swing;
		m_landed = m_time;
		swing = -1;
	}
	// Where a step lands: the capture point as it will be after a_left more seconds on the other foot (it runs away from
	// that foot meanwhile, so a faster body takes a longer step), on the foot's own side. Shorter as strength fades (from
	// where the step began: measured from the moving foot, it chased the weight into a lunge), never beyond the leg's
	// reach from where the hip will be by then, with the pelvis halfway down to kStaggerLow (reach measured with the
	// pelvis already that low left the foot hanging, leg straight, short of a spot it could not get down to).
	const Vec3  pelvis = Flat(pos(m_trunk[0].part));
	const float lowest = m_standHeight * kStaggerLow, reachHeight = 0.5f * (std::max(pos(m_trunk[0].part).GetZ() - m_ground, lowest) + lowest);
	const auto  spotFor = [&](int a_foot, float a_left) {
		const auto& f = m_feet[a_foot];
		const Vec3  from = foot[1 - a_foot], start = a_foot == swing ? Flat(m_swingFrom) : foot[a_foot];
		Vec3        to = from + (capture - from) * std::exp(omega * a_left) + right * f.offset.GetX();
		to = start + Clamp(to - start, g_plugin.staggerStep * a_strength * (1.0f + 0.5f * speed));  // faster: longer (to the leg's reach)
		const Vec3  hip = pelvis + Flat(vel) * a_left + right * f.offset.GetX();
		const float drop = reachHeight - f.offset.GetZ(), leg = kStraight * f.leg;
		to = hip + Clamp(to - hip, std::sqrt(std::max(0.0f, leg * leg - drop * drop)));
		to.SetZ(m_ground + f.offset.GetZ());
		return OwnSide(a_world, to, a_foot, right);
	};
	// A step starts when the capture point leaves the feet, or at once for a foot the hit kicked up. Otherwise a foot left
	// far from under its hip steps back in: the body stopped over spread feet stood in a straddle and sank into a split.
	// A shot leg folded up on purpose (below) counts as standing here, so the feet still alternate.
	const bool stands[2]{ down[0] || hurt == 0, down[1] || hurt == 1 };
	if (swing < 0 && a_strength > kStaggerWeakest) {
		int pick = -1;
		if (stands[0] != stands[1]) pick = stands[0] ? 1 : 0;
		else if (stands[0] && m_time - m_landed >= 0.25f * cfg.stepTime) {
			const Vec3  ab = foot[1] - foot[0];
			const float t = ab.LengthSq() > 1.0e-6f ? std::clamp((capture - foot[0]).Dot(ab) / ab.LengthSq(), 0.0f, 1.0f) : 0.0f;
			const float out[2]{ (foot[0] - pelvis - right * m_feet[0].offset.GetX()).Length(), (foot[1] - pelvis - right * m_feet[1].offset.GetX()).Length() };
			if ((capture - (foot[0] + ab * t)).Length() > kStaggerTrigger * (2.0f - a_strength)) {  // weaker: reacts later
				// Feet alternate, as in Walk; the first step goes with the foot that has farther to go. (Always the farther:
				// one leg stepped again and again while the other trailed far out, straight.)
				const float time = cfg.stepTime / (1.0f + 0.5f * speed);
				pick = m_lastSwing >= 0 ? 1 - m_lastSwing : Flat(spotFor(0, time) - foot[0]).Length() > Flat(spotFor(1, time) - foot[1]).Length() ? 0 : 1;
			} else if (std::max(out[0], out[1]) > kStaggerWide) {
				pick = out[0] > out[1] ? 0 : 1;
			}
		}
		if (pick >= 0) {
			swing = pick;
			m_swingStart = m_time;
			m_swingTime = cfg.stepTime / ((1.0f + 0.5f * speed) * (0.5f + 0.5f * a_strength));  // quicker when moving fast, slower when weak
			m_swingFrom = pos(m_feet[pick].part);
		}
	}
	std::vector<int> planted;
	for (int i = 0; i < 2; ++i)
		if (i != swing && down[i]) planted.push_back(m_feet[i].part);
	// A shot leg bears no weight while the other foot is down: the body lurches over onto it.
	if (planted.size() == 2 && hurt >= 0) std::erase(planted, m_feet[hurt].part);
	if (planted.empty()) return false;  // airborne: limp until a foot comes down
	// And it gives: the knee folds and the foot comes up where it is. Left on the floor, the straight leg propped the body
	// up as a strut and it stood on it. (Then it is put down again as a step, and gives again: a hobble.) So does a leg
	// left far out once the body is sinking: held straight, it slid out into a split.
	const bool sinking = pos(m_trunk[0].part).GetZ() - m_ground < kStaggerSplit * m_standHeight;
	for (int i = 0; i < 2; ++i) {
		const bool shot = i == hurt && down[i] && planted[0] != m_feet[i].part;
		const bool left = sinking && down[i] && (foot[i] - pelvis - right * m_feet[i].offset.GetX()).Length() > kStaggerFar;
		if (i == swing || !(shot || left)) continue;
		Vec3 ankle = left ? 0.5f * (foot[i] + pelvis + right * m_feet[i].offset.GetX()) : foot[i];  // a far one is drawn in, or it lifts straight
		ankle.SetZ(m_ground + m_feet[i].offset.GetZ() + kHurtLift);
		bodies.AddImpulse(a_world.Body(m_trunk[0].part), -DriveLeg(a_world, i, ankle, Vec3::sAxisZ().Cross(right), a_dt));
	}
	if (swing >= 0) {
		// Sped up if the body has: the step that starts with the hit is timed before the hit's push lands.
		m_swingTime = std::min(m_swingTime, cfg.stepTime / ((1.0f + 0.5f * speed) * (0.5f + 0.5f * a_strength)));
		spot[swing] = spotFor(swing, std::max(0.0f, m_swingTime - (m_time - m_swingStart)));
		Swing(a_world, spot[swing], m_swingTime, right, a_dt);
	}

	// Lift: as in key 4, only while the weight is over the feet (mid-step: between the standing foot and where the
	// stepping one lands), times a_legs; a staggering leg leans out further than a standing one. Height: as high as a
	// nearly straight leg reaches from each planted foot (a pendulum's arc), sinking toward kBuckled as a_legs fades (the
	// knees give way).
	const Vec3  a = Flat(pos(planted.front())), b = swing >= 0 ? Flat(spot[swing]) : Flat(pos(planted.back())), ab = b - a;
	const float along = ab.LengthSq() > 1.0e-6f ? std::clamp((Flat(com) - a).Dot(ab) / ab.LengthSq(), 0.0f, 1.0f) : 0.0f;
	m_support = a_legs * (1.0f - Smooth(((Flat(com) - (a + ab * along)).Length() - kStaggerBalanced) / kStaggerToppled));
	if (planted.size() == 1 && hurt >= 0 && planted[0] == m_feet[hurt].part) m_support *= g_plugin.staggerHurtLeg;  // the shot leg buckles under it
	float reach = m_standHeight;
	for (int i = 0; i < 2; ++i) {
		if (i == swing || !down[i]) continue;
		const float d = (pelvis - foot[i] + right * m_feet[i].offset.GetX()).Length(), leg = kStraight * m_feet[i].leg;
		reach = std::min(reach, m_feet[i].offset.GetZ() + std::sqrt(std::max(0.0f, leg * leg - d * d)));
	}
	const float buckled = m_standHeight * kBuckled;
	base = Vec3(pelvis.GetX(), pelvis.GetY(), m_ground + buckled + (std::max(reach, lowest) - buckled) * a_legs);

	// Sideways: the ground pushes the weight from a centre of pressure under the planted feet, which the legs pick to
	// brake (a_brake) as far as the feet allow, with the share of the weight they carry; within friction.
	// The legs fail to hold it: the centre of pressure lags a_tip behind the weight the way it was going (and toward a
	// shot leg), so it never settles upright: it tips over and the steps have to chase it.
	// (a shot leg's side counts double: thrown onto the good leg, the two cancelled and it stood on it)
	const Vec3 tip = (fallDir + (hurt >= 0 ? right * (m_feet[hurt].offset.GetX() < m_feet[1 - hurt].offset.GetX() ? -2.0f : 2.0f) : Vec3::sZero()))
	                     .NormalizedOr(Vec3::sZero());
	const Vec3 want = Flat(com) + Flat(vel) * (a_brake * a_strength / (omega * omega)) - tip * a_tip;
	Vec3       cop = a;
	if (planted.size() == 2) {
		const Vec3  pq = Flat(pos(planted.back())) - a;
		const float t = pq.LengthSq() > 1.0e-6f ? std::clamp((want - a).Dot(pq) / pq.LengthSq(), 0.0f, 1.0f) : 0.0f;
		cop = a + pq * t;
	}
	cop += Clamp(want - cop, kSole);
	const Vec3 push = Clamp((Flat(com) - cop) * (omega * omega * m_support), kStaggerFriction * g * m_support);
	// The trunk springs only keep its shape and hold it up (aimed at where the pelvis is, moving as the weight moves);
	// the net sideways push is then made exactly the ground's. The trunk slumps toward the tip as well: held straight over
	// feet set across the tip, the centre of pressure (half a sole of room) could not tip it and it just stood.
	const Quat slump = tip.IsNearZero() ? Quat::sIdentity() : Quat::sRotation(Vec3::sAxisZ().Cross(tip).Normalized(), g_plugin.staggerSlump * a_tip);
	Vec3       impulse = HoldTrunk(a_world, base, slump * facing, Vec3(vel.GetX(), vel.GetY(), 0.0f), a_dt);
	const Vec3 extra = (push * (mass * a_dt) - Flat(impulse)) / trunkMass;
	for (const auto& t : m_trunk) bodies.AddLinearVelocity(a_world.Body(t.part), extra);
	impulse += extra * trunkMass;
	Brace(a_world, impulse, planted, a_dt);
	const float hold = std::min(1.0f, cfg.stanceHold * a_dt);
	for (int i = 0; i < 2; ++i) {
		if (i == swing || !down[i]) continue;
		const auto calf = a_world.Body(m_feet[i].calf);
		bodies.AddLinearVelocity(calf, -Flat(Vec3(bodies.GetLinearVelocity(calf))) * hold);
	}
	return true;
}

// Getting up after a fall. FO4's hips bend only ~30-45 degrees and the knee one way, so no squatting or sitting up:
// lie still -> roll onto the belly -> pivot up about the knees to kneeling -> swing one foot forward -> rise over it
// while the back foot steps in -> hand back to the walker. Support works as in walking: the trunk is held only through
// what touches the ground (shins, knees, feet, hands), which take the reaction.
bool Puppet::GetUp(World& a_world, float a_dt)
{
	auto&       bodies = a_world.Bodies();
	const auto& cfg = Lab::g_config.puppet;
	const auto  pos = [&](int a_part) { return Vec3(bodies.GetPosition(a_world.Body(a_part))); };
	const Vec3  pelvis = pos(m_trunk[0].part);
	const Quat  frame = BodyFrame(a_world);
	const Vec3  belly = frame * Vec3::sAxisY();
	const float height = pelvis.GetZ() - m_ground;
	constexpr float kHalfPi = 0.5f * std::numbers::pi_v<float>;
	const auto  begin = [&](Rise a_stage) {
		rise = a_stage;
		m_stageStart = m_time;
		m_stageFrom = pelvis;
	};

	if (rise == Rise::kNone) {
		// Down: pelvis low or trunk tipped over. Key 4 gives up sooner, at fGiveUpAngle, and falls.
		const float upright = weak && m_time - m_upAt > kRegainTime ? std::cos(Lab::g_config.puppet.giveUpAngle * std::numbers::pi_v<float> / 180.0f) : 0.5f;
		if (height > 0.55f * m_standHeight && (frame * Vec3::sAxisZ()).GetZ() > upright) return false;
		swing = -1;
		begin(Rise::kDown);  // the walker lets go: holding the trunk up by grounded feet would hoist a lying body
	}
	const float stage = m_time - m_stageStart;
	// Stages whose targets start away from the body ease in: a full-strength first frame kicks the limbs. The foot swings
	// start from where the foot is, and easing those only makes the foot lag and then whip to catch up.
	m_strength = rise == Rise::kRoll || rise == Rise::kKneel ? Smooth(stage / kEaseIn) : 1.0f;

	// Supports: every limb body touching the ground. The legs being moved are taken out below.
	std::vector<int> ground;
	for (int i = 0; i < a_world.PartCount(); ++i)
		if (!IsTrunk(i) && OnGround(a_world, i)) ground.push_back(i);
	const auto without = [&](int a_foot) {
		const auto& f = m_feet[a_foot];
		std::erase_if(ground, [&](int p) { return p == f.part || p == f.calf || p == f.thigh; });
	};
	const auto support = [&](Vec3 a_reaction) {
		if (!ground.empty()) Brace(a_world, a_reaction, ground, a_dt);
	};
	const Vec3  fwd = Facing(m_riseYaw) * Vec3::sAxisY(), right = Facing(m_riseYaw) * Vec3::sAxisX();
	const float kneeRadius = a_world.Part(m_feet[0].calf).radius;
	const float thigh = 0.5f * (m_feet[0].thighLength + m_feet[1].thighLength);
	const auto  startKneel = [&] {
		m_pivot = 0.5f * (Flat(pos(m_feet[0].calf)) + Flat(pos(m_feet[1].calf)));  // knees: the body pivots up about them
		const Vec3 lie = Flat(pelvis - m_pivot);
		m_riseYaw = YawOf(lie.Length() > 0.1f ? lie : Flat(pos(m_head) - pelvis));
		// Hands go down beside the chest: where each shoulder lies once face down (also from a roll that stopped on the side).
		const Vec3 back = Facing(m_riseYaw) * Vec3::sAxisY() * 0.1f;
		for (const int a : m_arms)
			if (Has(Lower(a_world.Part(a).name), { "upper" }))
				m_hands[Lower(a_world.Part(a).name)[0] == 'r'] = Flat(pelvis + Pitched(m_riseYaw, kHalfPi) * (frame.Conjugated() * (pos(a) - pelvis))) - back +
				                                                Vec3(0, 0, m_ground);
		begin(Rise::kKneel);
	};
	// Kneeling up: the knee-to-hip angle above the floor a_t into a rise lasting a_time, and the hips at that angle.
	const auto kneelAngle = [&](float a_t, float a_time) { return Smooth(a_t / a_time) * kHalfPi; };
	const auto kneelHips = [&](float a_theta) {
		Vec3 hips = m_pivot + (fwd * std::cos(a_theta) + Vec3::sAxisZ() * std::sin(a_theta)) * thigh;
		hips.SetZ(hips.GetZ() + m_ground + kneeRadius);
		return hips;
	};
	// A foot swung along a lifted arc (as in walking), its push taken from the pelvis.
	const auto swingFoot = [&](int a_foot, Vec3 a_to, float a_phase) {
		Vec3 arc = m_swingFrom + (a_to - m_swingFrom) * Smooth(a_phase);
		arc.SetZ(arc.GetZ() + cfg.stepLift * std::sin(std::numbers::pi_v<float> * std::min(1.0f, a_phase * 1.25f)));
		bodies.AddImpulse(a_world.Body(m_trunk[0].part), -DriveLeg(a_world, a_foot, arc, fwd, a_dt));
		without(a_foot);
	};

	// Arms straightened from the shoulders along a_dir(shoulder, side: 0 left, 1 right) on a slow spring (kArmKp): they
	// drift there instead of being flung. Returns the impulse given; arms moved this way stop counting as supports unless
	// a_planted (hands pushing on the floor: whichever touch it take their share).
	const auto moveArms = [&](auto a_dir, bool a_planted) {
		Vec3 impulse = Vec3::sZero();
		for (const int a : m_arms) {
			const auto name = Lower(a_world.Part(a).name);
			const auto shoulder = std::find_if(m_arms.begin(), m_arms.end(), [&](int b) {
				const auto other = Lower(a_world.Part(b).name);
				return other[0] == name[0] && Has(other, { "upper" });
			});
			if (shoulder == m_arms.end()) continue;
			const float reach = Has(name, { "upper" }) ? 0.15f : Has(name, { "hand" }) ? 0.6f : 0.4f;  // along the arm, m
			const auto  id = a_world.Body(a);
			const Vec3  from = pos(*shoulder);
			// Carried along with the shoulder (left behind, they drag the rising chest down), except in the roll, where arms
			// carried round with the chest prop it up instead; the drift on top is capped so a far-off target doesn't fling them.
			const Vec3 carried = rise == Rise::kRoll ? Vec3::sZero() : Vec3(bodies.GetPointVelocity(a_world.Body(*shoulder), from));
			const Vec3 drift = Clamp((from + a_dir(from, name[0] == 'r' ? 1 : 0) * reach - Vec3(bodies.GetCenterOfMassPosition(id))) * kArmKp, kArmSpeed);
			Vec3       dv = Clamp(drift + carried - Vec3(bodies.GetLinearVelocity(id)), cfg.swingAccel * m_strength * a_dt);
			dv.SetZ(dv.GetZ() + g_sim.gravity * a_dt);
			bodies.AddLinearVelocity(id, dv);
			impulse += dv * a_world.Part(a).mass;
		}
		std::erase_if(ground, [&](int p) {  // a planted hand pushes on the floor; the rest of the arm, pushed down, drags the chest
			return std::find(m_arms.begin(), m_arms.end(), p) != m_arms.end() && !(a_planted && Has(Lower(a_world.Part(p).name), { "hand" }));
		});
		return impulse;
	};
	// Hands down by the thighs, clear of them: the pose they hang in anyway, so letting go at the end does not swing them.
	const auto atSides = [&](Vec3 a_shoulder, int) {
		const Vec3 out = (a_shoulder - pelvis).Dot(right) > 0.0f ? right : -right;
		return (out * 0.1f - Vec3::sAxisZ()).Normalized();
	};

	switch (rise) {
	case Rise::kDown:  // limp until the body lies still
		if (stage < 0.5f || (std::max(Vec3(bodies.GetLinearVelocity(a_world.Body(m_trunk[0].part))).Length(),
		                               Vec3(bodies.GetLinearVelocity(a_world.Body(m_head))).Length()) > 0.5f && stage < 2.0f))  // head too: it lands last
			return true;
		m_riseYaw = YawOf(Flat(pos(m_head) - pelvis));
		if (belly.GetZ() < -0.2f) startKneel();  // already more face down than up
		else {
			// The roll: how far the belly has to turn about the body's length to face down, the way it already leans.
			// Angle measured from straight up toward the right; a turn of +a about the length axis adds a.
			const Vec3  r = Facing(m_riseYaw) * Vec3::sAxisX();
			const float now = std::atan2(belly.Dot(r), belly.GetZ());
			m_rollTurn = (now >= 0.0f ? std::numbers::pi_v<float> : -std::numbers::pi_v<float>) - now;
			begin(Rise::kRoll);
		}
		return true;

	case Rise::kRoll: {
		// The target turns steadily from where the body lay to face down (kRollTime per half turn), so the trunk is never
		// pulled far: aiming straight at the end pose snapped the body round and whipped the limbs.
		const float rollTime = kRollTime * std::abs(m_rollTurn) / std::numbers::pi_v<float>;
		const float s = Smooth(stage / rollTime);
		const Quat  target = Quat::sRotation(fwd, m_rollTurn * (s - 1.0f)) * Pitched(m_riseYaw, kHalfPi);
		Vec3        lying = Flat(pelvis);
		lying.SetZ(m_ground + a_world.Part(m_trunk[0].part).radius);
		if (ground.empty()) return true;
		Vec3 reaction = HoldTrunk(a_world, lying, target, Vec3::sZero(), a_dt);
		m_strength *= kRollLimbs;
		// Overhead, as in a log roll: the arm underneath otherwise props the body up. (Tried: the far arm reaching across,
		// which ends up under the chest, and over the head toward the roll, which drags the body across the floor.)
		reaction += moveArms([&](Vec3, int) { return fwd; }, false);
		// Until on its side: the far leg swings across the near one and turns the hips along. (Kept on past the side it
		// props the hips up instead.)
		if (belly.GetZ() > 0.3f) {
			const Vec3 toward = Flat(Quat::sRotation(fwd, m_rollTurn > 0.0f ? -kHalfPi : kHalfPi) * Pitched(m_riseYaw, kHalfPi) * Vec3::sAxisY());
			const int  far = pos(m_feet[0].thigh).Dot(toward) < pos(m_feet[1].thigh).Dot(toward) ? 0 : 1;  // by hip: feet swap once crossed
			bodies.AddImpulse(a_world.Body(m_trunk[0].part),
				-DriveLeg(a_world, far, pos(m_feet[1 - far].part) + toward * 0.2f + Vec3(0, 0, 0.15f), toward, a_dt));
			without(far);
		}
		support(reaction);
		if ((s >= 1.0f && belly.GetZ() < -0.5f) || stage > rollTime + 1.0f) startKneel();  // timed out: kneeling up still works from the side
		return true;
	}

	case Rise::kKneel: {  // hands push the chest up as thighs and trunk swing up about the knees; the shins stay down
		const float theta = kneelAngle(stage, kKneelTime);
		const Quat  target = Pitched(m_riseYaw, kHalfPi - theta);
		const Vec3  hips = kneelHips(theta);
		Vec3        reaction = Vec3::sZero();
		if (std::abs((frame * Vec3::sAxisX()).GetZ()) < 0.5f)  // not while still on its side (a timed-out roll): arms driven there pin it down
			reaction = moveArms([&](Vec3 a_shoulder, int a_side) {
				const Vec3 toHand = m_hands[a_side] - a_shoulder;
				return toHand.Length() < kArmReach ? toHand / kArmReach : atSides(a_shoulder, a_side);  // short: the elbow bends, the hand stays put
			}, true);
		if (!ground.empty()) {
			reaction += HoldTrunk(a_world, hips, target, Vec3::sZero(), a_dt);
			for (const auto& f : m_feet) {  // each thigh along its knee-to-hip line
				const auto id = a_world.Body(f.thigh);
				const Vec3 knee = pos(f.calf), hip = hips + target * Vec3(f.offset.GetX(), 0.0f, 0.0f);
				Vec3       dv = Clamp((hip + (knee - hip) * f.thighCom - Vec3(bodies.GetCenterOfMassPosition(id))) * cfg.swingKp -
				                          Vec3(bodies.GetLinearVelocity(id)),
					cfg.swingAccel * m_strength * a_dt);
				dv.SetZ(dv.GetZ() + g_sim.gravity * a_dt);
				bodies.AddLinearVelocity(id, dv);
				reaction += dv * a_world.Part(f.thigh).mass;
			}
		}
		support(reaction);
		if ((stage > kLungeStart * kKneelTime && (frame * Vec3::sAxisZ()).GetZ() > 0.9f) || stage > kKneelTime + 0.3f) {  // early once up
			if ((frame * Vec3::sAxisZ()).GetZ() < 0.8f) begin(Rise::kDown);  // didn't make it: lie still and retry
			else {
				m_front = 1 - static_cast<int>(m_lunges++ % 2);  // alternate the leading leg every attempt: a retry tries the other one
				m_swingFrom = pos(m_feet[m_front].part);
				begin(Rise::kLunge);
			}
		}
		return true;
	}

	case Rise::kLunge: {  // the hips finish kneeling up while one foot swings forward and plants
		const auto& f = m_feet[m_front];
		const float phase = std::min(1.0f, stage / kLungeTime);
		Vec3        to = m_pivot + fwd * kLungeReach + right * f.offset.GetX();
		to.SetZ(m_ground + f.offset.GetZ());
		swingFoot(m_front, to, phase);
		// As the foot comes down, the hand on its side goes to its knee, ready to push on it.
		const Vec3 knee = pos(f.calf);
		Vec3       reaction = moveArms([&](Vec3 a_shoulder, int a_side) {
			return phase > 0.5f && (a_shoulder - pelvis).Dot(right) * f.offset.GetX() > 0.0f ? (knee - a_shoulder).Normalized() : atSides(a_shoulder, a_side);
		}, false);
		const Vec3 hips = kneelHips(kneelAngle(stage + kLungeStart * kKneelTime, kKneelTime));  // the kneel carried on
		if (!ground.empty()) support(reaction + HoldTrunk(a_world, hips, Facing(m_riseYaw), Vec3::sZero(), a_dt));
		if (phase >= 0.9f && OnGround(a_world, f.part)) begin(Rise::kStand);  // down: rise without waiting for the swing to finish
		else if (stage > 3.0f * kLungeTime || (frame * Vec3::sAxisZ()).GetZ() < 0.7f)
			begin(Rise::kDown);  // foot never got down, or toppling over (held up, it was hauled off the floor): lie still and retry
		return true;
	}

	case Rise::kStand: {  // the pelvis pushes up over the front foot, chest forward, straightening last; the back foot steps in
		const float s = Effort(stage / kStandTime);
		const auto& front = m_feet[m_front];
		const auto& back = m_feet[1 - m_front];
		Vec3        over = Flat(pos(front.part)) - right * front.offset.GetX() - fwd * front.offset.GetY();
		over.SetZ(m_ground + m_standHeight * cfg.crouch);
		const Vec3 knee = pos(front.calf);
		Vec3       reaction = moveArms([&](Vec3 a_shoulder, int a_side) {  // pushing on the front knee for the first part
			return stage < 0.4f * kStandTime && (a_shoulder - pelvis).Dot(right) * front.offset.GetX() > 0.0f ? (knee - a_shoulder).Normalized() :
			                                                                                                   atSides(a_shoulder, a_side);
		}, false);
		if (stage / kStandTime > 0.4f) {
			const float phase = std::min(1.0f, (stage / kStandTime - 0.4f) / 0.6f);
			if (m_swingFrom.GetZ() > 1.0e8f) m_swingFrom = pos(back.part);
			Vec3 to = over + right * back.offset.GetX() + fwd * back.offset.GetY();
			to.SetZ(m_ground + back.offset.GetZ());
			swingFoot(1 - m_front, to, phase);
		} else {
			m_swingFrom = Vec3::sReplicate(1.0e9f);  // set when the back foot starts
		}
		// Chest forward over the front foot while rising, straightened last. Not before: with the front thigh still forward
		// the hip is at its limit (~45 degrees) and leaning then wrenches the legs about.
		const Quat upright = Pitched(m_riseYaw, kLean * std::sin(std::numbers::pi_v<float> * std::clamp((stage / kStandTime - 0.2f) / 0.75f, 0.0f, 1.0f)));
		if (!ground.empty()) support(reaction + HoldTrunk(a_world, m_stageFrom + (over - m_stageFrom) * s, upright, Vec3::sZero(), a_dt));
		if (stage > kStandTime && height > 0.8f * m_standHeight) {  // up: the walker takes over from here
			m_root = pelvis;
			yaw = m_riseYaw;
			swing = m_lastSwing = -1;
			m_landed = m_time;
			rise = Rise::kNone;
			++getUps;
			m_upAt = m_time;
			return false;
		}
		if (stage > kStandTime + 2.0f || (frame * Vec3::sAxisZ()).GetZ() < 0.7f) begin(Rise::kDown);
		return true;
	}

	default: return false;
	}
}
