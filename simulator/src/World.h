#pragma once

// The Jolt side: a flat floor plus one ragdoll built from a Fallout 4 dump (human_ragdoll.json).
// Units and axes match Havok in the game: metres, Z up.

#include <Jolt/Jolt.h>

#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "Config.h"

struct SimSettings
{
	float timeScale = 1.0f, gravity = 9.81f, friction = 0.5f, restitution = 0.0f;
	float linearDamping = 0.05f, angularDamping = 0.05f, velocitySteps = 10.0f, positionSteps = 2.0f, pushSpeed = 3.0f;
};
inline SimSettings g_sim;

inline std::vector<Lab::Setting> SimTable(SimSettings& s)
{
	return {
		{ "Sim", "fTimeScale", &s.timeScale, 0.05f, 2, "Simulator only. Slow motion (1 = real time)." },
		{ "Sim", "fGravity", &s.gravity, 0, 30, "m/s^2." },
		{ "Sim", "fFriction", &s.friction, 0, 2, "Body and floor friction (FO4's value is not traced yet)." },
		{ "Sim", "fRestitution", &s.restitution, 0, 1, "Bounciness." },
		{ "Sim", "fLinearDamping", &s.linearDamping, 0, 5, "Per-body linear damping, 1/s (FO4's value is not traced yet)." },
		{ "Sim", "fAngularDamping", &s.angularDamping, 0, 5, "Per-body angular damping, 1/s." },
		{ "Sim", "fVelocitySteps", &s.velocitySteps, 1, 50, "Solver velocity iterations." },
		{ "Sim", "fPositionSteps", &s.positionSteps, 1, 20, "Solver position iterations." },
		{ "Sim", "fPushSpeed", &s.pushSpeed, 0, 20, "F key: m/s added to the chest along the camera." },
	};
}

struct PartInfo
{
	std::string name;   // as in the game, e.g. "LLeg_Calf"
	float       radius{};
	JPH::Vec3   a, b;   // capsule segment, body-local
	float       mass{};
	int         parent{ -1 };  // part this one is jointed to (-1 = root)
};

class World
{
public:
	// Key T. Everything but the flat floor is a static mesh on top of it, laid out ahead of the dumped pose (a flat pad
	// around the start, so Reset never spawns into it).
	enum class Terrain { kFlat, kSlope, kStairs, kUneven, kCount };
	static const char* TerrainName(Terrain a_terrain);
	using Tri = std::array<JPH::Vec3, 3>;  // counter-clockwise seen from outside (both Jolt and raylib cull the back)

	World();
	~World();

	bool Load(const std::string& a_jsonPath, std::string& a_error);  // reads the dump; call once
	void Reset();                                                    // ragdoll back to the dumped standing pose
	void Step(float a_dt);
	void ApplySettings();                                            // g_sim -> live bodies and solver
	void SetTerrain(Terrain a_terrain);
	Terrain                 GetTerrain() const { return m_terrainKind; }
	const std::vector<Tri>& TerrainTris() const { return m_terrainTris; }
	float GroundZ(float a_x, float a_y) const;  // height of the floor or terrain under a point (the plugin's ground ray)

	JPH::BodyInterface& Bodies() { return m_system.GetBodyInterface(); }
	JPH::BodyID         Body(int a_part) const { return m_ragdoll->GetBodyID(a_part); }
	int                 PartCount() const { return static_cast<int>(m_parts.size()); }
	const PartInfo&     Part(int a_part) const { return m_parts[a_part]; }
	float               StartYaw() const { return m_startYaw; }  // game heading the dumped pose faces
	int                 Pick(JPH::Vec3 a_from, JPH::Vec3 a_dir) const;    // first ragdoll part along the ray, -1 = none
	JPH::Mat44          InvInertia(int a_part);                           // world-space inverse inertia, right now

private:
	static bool InitJolt();  // allocator, factory, types: before any Jolt member below is constructed

	bool                                                     m_init = InitJolt();
	JPH::ObjectLayerPairFilterTable                       m_pairs{ 2 };
	JPH::BroadPhaseLayerInterfaceTable                      m_bpLayers{ 2, 2 };
	std::unique_ptr<JPH::ObjectVsBroadPhaseLayerFilterTable> m_objVsBp;
	std::unique_ptr<JPH::TempAllocatorImpl>                  m_temp;
	std::unique_ptr<JPH::JobSystemThreadPool>                m_jobs;
	JPH::PhysicsSystem                                       m_system;
	JPH::BodyID                                              m_floor, m_terrain;
	Terrain                                                  m_terrainKind{ Terrain::kFlat };
	std::vector<Tri>                                         m_terrainTris;
	JPH::Ref<JPH::RagdollSettings>                           m_settings;
	JPH::Ref<JPH::Ragdoll>                                   m_ragdoll;
	std::vector<PartInfo>                                    m_parts;
	float                                                    m_startYaw{};
};
