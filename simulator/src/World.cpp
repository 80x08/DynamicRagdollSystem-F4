#include "World.h"

#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Skeleton/Skeleton.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <numbers>
#include <thread>

using namespace JPH;

namespace
{
	constexpr ObjectLayer kStatic = 0, kMoving = 1;

	Vec3 V3(const nlohmann::json& a_j) { return { a_j[0].get<float>(), a_j[1].get<float>(), a_j[2].get<float>() }; }

	// Havok stores rotations as three column vectors.
	Mat44 Columns(const nlohmann::json& a_j)
	{
		return { Vec4(V3(a_j[0]), 0), Vec4(V3(a_j[1]), 0), Vec4(V3(a_j[2]), 0), Vec4(0, 0, 0, 1) };
	}

	bool InRange(float a_angle, float a_min, float a_max) { return a_angle >= a_min - 0.05f && a_angle <= a_max + 0.05f; }

	// Sign conventions between Havok and Jolt limits are not verified. The dumped pose is a real, valid pose, so if
	// the current angle only fits the range mirrored, mirror it. shortcut: pose-based guess; upgrade by tracing
	// hkpLimitedHingeConstraintData's angle in Ghidra if a limit looks wrong in the simulator.
	void FitRange(float a_angle, float& a_min, float& a_max, const char* a_what, const std::string& a_joint)
	{
		if (InRange(a_angle, a_min, a_max) || !InRange(-a_angle, -a_max, -a_min)) return;
		std::printf("  %s %s: pose angle %.2f only fits mirrored, using [%.2f, %.2f]\n", a_joint.c_str(), a_what, a_angle, -a_max, -a_min);
		std::swap(a_min, a_max);
		a_min = -a_min;
		a_max = -a_max;
	}

	float SignedAngle(Vec3 a_from, Vec3 a_to, Vec3 a_axis) { return std::atan2(a_from.Cross(a_to).Dot(a_axis), a_from.Dot(a_to)); }
}

bool World::InitJolt()
{
	static const bool done = [] {
		RegisterDefaultAllocator();
		Factory::sInstance = new Factory();
		RegisterTypes();
		return true;
	}();
	return done;
}

World::World()
{
	m_pairs.EnableCollision(kMoving, kStatic);
	m_pairs.EnableCollision(kMoving, kMoving);
	m_bpLayers.MapObjectToBroadPhaseLayer(kStatic, BroadPhaseLayer(0));
	m_bpLayers.MapObjectToBroadPhaseLayer(kMoving, BroadPhaseLayer(1));
	m_objVsBp = std::make_unique<ObjectVsBroadPhaseLayerFilterTable>(m_bpLayers, 2, m_pairs, 2);
	m_temp = std::make_unique<TempAllocatorImpl>(16 * 1024 * 1024);
	m_jobs = std::make_unique<JobSystemThreadPool>(cMaxPhysicsJobs, cMaxPhysicsBarriers, std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 1));
	m_system.Init(1024, 0, 1024, 1024, m_bpLayers, *m_objVsBp, m_pairs);

	BodyCreationSettings floor(new BoxShape(Vec3(50, 50, 0.5f)), RVec3(0, 0, -0.5f), Quat::sIdentity(), EMotionType::Static, kStatic);
	m_floor = Bodies().CreateAndAddBody(floor, EActivation::DontActivate);
}

World::~World()
{
	if (m_ragdoll) m_ragdoll->RemoveFromPhysicsSystem();
	m_ragdoll = nullptr;
	Bodies().RemoveBody(m_floor);
	Bodies().DestroyBody(m_floor);
}

bool World::Load(const std::string& a_jsonPath, std::string& a_error)
{
	nlohmann::json doc;
	try {
		std::ifstream file{ a_jsonPath };
		if (!file) {
			a_error = "cannot open " + a_jsonPath;
			return false;
		}
		doc = nlohmann::json::parse(file);
	} catch (const std::exception& e) {
		a_error = e.what();
		return false;
	}
	const auto& bodies = doc["bodies"];
	const auto  count = static_cast<int>(bodies.size());

	// Body poses: centre the pelvis (part 0) over the origin and put the lowest capsule on the floor.
	std::vector<Mat44> rot(count);
	std::vector<Vec3>  pos(count);
	float              lowest = 1.0e9f;
	m_parts.clear();
	for (int i = 0; i < count; ++i) {
		const auto& b = bodies[i];
		rot[i] = Columns(b["rot"]);
		pos[i] = V3(b["pos"]);
		m_parts.push_back({ b["name"].get<std::string>(), b["radius"].get<float>(), V3(b["a"]), V3(b["b"]), b["mass"].get<float>(), b["skelParent"].get<int>() });
		const auto& p = m_parts.back();
		lowest = std::min({ lowest, (pos[i] + rot[i].Multiply3x3(p.a)).GetZ() - p.radius, (pos[i] + rot[i].Multiply3x3(p.b)).GetZ() - p.radius });
	}
	const Vec3 shift(-pos[0].GetX(), -pos[0].GetY(), 0.005f - lowest);
	for (auto& p : pos) p += shift;

	// Facing: the toes. A foot capsule runs heel (b) -> toe (a).
	Vec3 toes = Vec3::sZero();
	for (int i = 0; i < count; ++i)
		if (m_parts[i].name.find("Foot") != std::string::npos) toes += rot[i].Multiply3x3(m_parts[i].a - m_parts[i].b);
	m_startYaw = std::atan2(toes.GetX(), toes.GetY());

	auto skeleton = new Skeleton;
	m_settings = new RagdollSettings;
	m_settings->mSkeleton = skeleton;
	m_settings->mParts.resize(count);
	for (int i = 0; i < count; ++i) {
		const auto& info = m_parts[i];
		skeleton->AddJoint(info.name, bodies[i]["skelParent"].get<int>());

		// Havok capsule = segment a-b + radius; Jolt's runs along Y around the origin.
		const Vec3       d = info.b - info.a;
		const float      len = d.Length();
		RefConst<Shape>  core = len > 1.0e-4f ? static_cast<Shape*>(new CapsuleShape(0.5f * len, info.radius)) : new SphereShape(info.radius);
		const Quat       align = len > 1.0e-4f ? Quat::sFromTo(Vec3::sAxisY(), d / len) : Quat::sIdentity();
		auto&            part = m_settings->mParts[i];
		part.SetShape(new RotatedTranslatedShape(0.5f * (info.a + info.b), align, core));
		part.mPosition = RVec3(pos[i]);
		part.mRotation = rot[i].GetQuaternion().Normalized();
		part.mMotionType = EMotionType::Dynamic;
		part.mObjectLayer = kMoving;
		part.mAllowSleeping = false;
		// shortcut: inertia from the capsule, not the game's (motion frame differs from the body frame); fine until
		// spin behaviour needs to match exactly.
		part.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
		part.mMassPropertiesOverride.mMass = info.mass;
	}

	std::printf("Joints (fitting limit signs to the dumped pose):\n");
	for (const auto& c : doc["constraints"]) {
		const int child = c["child"], parent = c["parent"];
		if (child <= 0 || child >= count || bodies[child]["skelParent"].get<int>() != parent) {
			a_error = "constraint does not follow the skeleton";
			return false;
		}
		const auto  rotA = Columns(c["rotA"]), rotB = Columns(c["rotB"]);  // A = child, B = parent (Havok)
		const Vec3  twistC = rot[child].Multiply3x3(rotA.GetAxisX()), planeC = rot[child].Multiply3x3(rotA.GetAxisY());
		const Vec3  twistP = rot[parent].Multiply3x3(rotB.GetAxisX()), planeP = rot[parent].Multiply3x3(rotB.GetAxisY());
		const Vec3  pivotC = pos[child] + rot[child].Multiply3x3(V3(c["pivotA"]));
		const Vec3  pivotP = pos[parent] + rot[parent].Multiply3x3(V3(c["pivotB"]));
		const float friction = c["friction"];
		const auto& name = m_parts[child].name;

		if (c["type"] == "hinge") {
			auto* h = new HingeConstraintSettings;
			h->mSpace = EConstraintSpace::WorldSpace;
			h->mPoint1 = RVec3(pivotP);
			h->mPoint2 = RVec3(pivotC);
			h->mHingeAxis1 = twistP;
			h->mHingeAxis2 = twistC;
			h->mNormalAxis1 = planeP;
			h->mNormalAxis2 = planeC;
			float lo = c["limit"][0], hi = c["limit"][1];
			FitRange(SignedAngle(planeP, planeC, twistP), lo, hi, "hinge", name);
			h->mLimitsMin = lo;
			h->mLimitsMax = hi;
			h->mMaxFrictionTorque = friction;
			m_settings->mParts[child].mToParent = h;
		} else {
			// Havok ragdoll joint = twist + cone + asymmetric "planes" limit. Jolt's six-DOF joint with pyramid swing
			// takes asymmetric limits: X = twist, Y = swing toward the normal (cone), Z = swing toward the plane axis
			// (Havok planes). shortcut: pyramid, not a true cone; fine for a stand-in.
			auto* s = new SixDOFConstraintSettings;
			s->mSpace = EConstraintSpace::WorldSpace;
			s->mPosition1 = RVec3(pivotP);
			s->mPosition2 = RVec3(pivotC);
			s->mAxisX1 = twistP;
			s->mAxisY1 = planeP;
			s->mAxisX2 = twistC;
			s->mAxisY2 = planeC;
			s->mSwingType = ESwingType::Pyramid;
			s->MakeFixedAxis(SixDOFConstraintSettings::TranslationX);
			s->MakeFixedAxis(SixDOFConstraintSettings::TranslationY);
			s->MakeFixedAxis(SixDOFConstraintSettings::TranslationZ);

			// Current angles, for the sign check: child frame in parent frame, then swing/twist.
			const Mat44 frameP(Vec4(twistP, 0), Vec4(planeP, 0), Vec4(twistP.Cross(planeP), 0), Vec4(0, 0, 0, 1));
			const Mat44 frameC(Vec4(twistC, 0), Vec4(planeC, 0), Vec4(twistC.Cross(planeC), 0), Vec4(0, 0, 0, 1));
			const Quat  q = (frameP.GetQuaternion().Conjugated() * frameC.GetQuaternion()).Normalized();
			Quat        swing, twist;
			q.GetSwingTwist(swing, twist);
			const float twistAngle = 2.0f * std::atan2(twist.GetX(), twist.GetW());
			const float swingZ = 2.0f * std::atan2(swing.GetZ(), swing.GetW());

			float cone = c["cone"], tLo = c["twist"][0], tHi = c["twist"][1], pLo = c["plane"][0], pHi = c["plane"][1];
			FitRange(twistAngle, tLo, tHi, "twist", name);
			FitRange(swingZ, pLo, pHi, "plane", name);
			s->SetLimitedAxis(SixDOFConstraintSettings::RotationX, tLo, tHi);
			s->SetLimitedAxis(SixDOFConstraintSettings::RotationY, -cone, cone);
			s->SetLimitedAxis(SixDOFConstraintSettings::RotationZ, std::max(pLo, -cone), std::min(pHi, cone));
			for (auto axis : { SixDOFConstraintSettings::RotationX, SixDOFConstraintSettings::RotationY, SixDOFConstraintSettings::RotationZ })
				s->mMaxFriction[axis] = friction;
			m_settings->mParts[child].mToParent = s;
		}
	}
	m_settings->DisableParentChildCollisions();  // FO4's filter: same-ragdoll bodies collide unless adjacent
	m_settings->CalculateBodyIndexToConstraintIndex();
	m_settings->CalculateConstraintIndexToBodyIdxPair();
	std::printf("Loaded %d bodies, %d joints from %s\n", count, static_cast<int>(doc["constraints"].size()), a_jsonPath.c_str());
	Reset();
	return true;
}

void World::Reset()
{
	if (m_ragdoll) m_ragdoll->RemoveFromPhysicsSystem();
	m_ragdoll = m_settings->CreateRagdoll(0, 0, &m_system);
	m_ragdoll->AddToPhysicsSystem(EActivation::Activate);
	ApplySettings();
}

void World::ApplySettings()
{
	m_system.SetGravity(Vec3(0, 0, -g_sim.gravity));
	auto physics = m_system.GetPhysicsSettings();
	physics.mNumVelocitySteps = static_cast<uint>(g_sim.velocitySteps);
	physics.mNumPositionSteps = static_cast<uint>(g_sim.positionSteps);
	m_system.SetPhysicsSettings(physics);
	Bodies().SetFriction(m_floor, g_sim.friction);
	if (!m_ragdoll) return;
	for (int i = 0; i < PartCount(); ++i) {
		BodyLockWrite lock(m_system.GetBodyLockInterface(), Body(i));
		if (!lock.Succeeded()) continue;
		auto& body = lock.GetBody();
		body.SetFriction(g_sim.friction);
		body.SetRestitution(g_sim.restitution);
		body.GetMotionProperties()->SetLinearDamping(g_sim.linearDamping);
		body.GetMotionProperties()->SetAngularDamping(g_sim.angularDamping);
	}
}

void World::Step(float a_dt)
{
	m_system.Update(a_dt, 1, m_temp.get(), m_jobs.get());
}

int World::Pick(Vec3 a_from, Vec3 a_dir) const
{
	RayCastResult hit;
	if (!m_system.GetNarrowPhaseQuery().CastRay(RRayCast(RVec3(a_from), a_dir * 100.0f), hit, {}, {}, IgnoreSingleBodyFilter(m_floor))) return -1;
	for (int i = 0; i < PartCount(); ++i)
		if (Body(i) == hit.mBodyID) return i;
	return -1;
}

Mat44 World::InvInertia(int a_part)
{
	BodyLockRead lock(m_system.GetBodyLockInterface(), Body(a_part));
	if (!lock.Succeeded()) return Mat44::sZero();
	const auto& body = lock.GetBody();
	return body.GetMotionProperties()->GetInverseInertiaForRotation(Mat44::sRotation(body.GetRotation()));
}
