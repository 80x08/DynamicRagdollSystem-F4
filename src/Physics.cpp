#include "Common.h"

namespace DynamicRagdollSystem
{

	RE::bhkWorld* GetWorld(RE::Actor* a_actor)
	{
		const auto cell = a_actor ? a_actor->GetParentCell() : nullptr;
		return cell ? cell->GetbhkWorld() : nullptr;
	}


	bool SafeRead(std::uintptr_t a_address, void* a_out, std::size_t a_size)
	{
		if (a_address < 0x10000 || a_address >= 0x800000000000ULL ||
			a_size == 0 || a_size > 0x800000000000ULL - a_address) {
			return false;
		}
		SIZE_T bytesRead = 0;
		return ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<const void*>(a_address), a_out, a_size, &bytesRead) != 0 &&
			bytesRead == a_size;
	}


	float DecodePackedFloat(std::uint16_t packed)
	{
		const std::uint32_t bits = static_cast<std::uint32_t>(packed) << 16;
		float value = 0.0f;
		std::memcpy(&value, &bits, sizeof(value));
		return value;
	}


	bool ReadMotionState(RE::hknpBSWorld* a_world, const RE::hknpBody& a_body, MotionState& a_out)
	{
		a_out = {};
		if (!a_body.m_flags.all(RE::hknpCollisionFlagsEnum::kIsDynamic)) {
			return false;
		}
		auto* motion = a_world->AccessMotion(a_body.m_motionId);
		if (!motion) {
			return false;
		}
		std::array<std::uint8_t, 0x60> raw{};
		if (!SafeRead(reinterpret_cast<std::uintptr_t>(motion), raw.data(), raw.size())) {
			return false;
		}
		std::memcpy(&a_out.orientation, raw.data() + 0x10, 0x10);
		std::uint16_t packed[4]{};
		std::memcpy(packed, raw.data() + 0x20, sizeof(packed));
		for (int i = 0; i < 3; ++i) {
			a_out.inverseInertia[i] = DecodePackedFloat(packed[i]);
		}
		a_out.inverseMass = DecodePackedFloat(packed[3]);
		std::memcpy(&a_out.linear, raw.data() + 0x40, 0x0C);
		std::memcpy(&a_out.angular, raw.data() + 0x50, 0x0C);
		const float qnorm = a_out.orientation.x * a_out.orientation.x + a_out.orientation.y * a_out.orientation.y +
			a_out.orientation.z * a_out.orientation.z + a_out.orientation.w * a_out.orientation.w;
		if (!std::isfinite(qnorm) || qnorm < 0.999f || qnorm > 1.001f ||
			!std::isfinite(a_out.linear.x) || !std::isfinite(a_out.linear.y) || !std::isfinite(a_out.linear.z) ||
			!std::isfinite(a_out.angular.x) || !std::isfinite(a_out.angular.y) || !std::isfinite(a_out.angular.z) ||
			!std::isfinite(a_out.inverseMass) || a_out.inverseMass <= 1.0e-8f || a_out.inverseMass > 100.0f) {
			return false;
		}
		for (const auto inertia : a_out.inverseInertia) {
			if (!std::isfinite(inertia) || inertia <= 1.0e-8f || inertia > 1.0e8f) {
				return false;
			}
		}
		return true;
	}


	Vec3 BodyPosition(const RE::hknpBody& a_body)
	{
		return { a_body.m_transform.m_translation.x, a_body.m_transform.m_translation.y, a_body.m_transform.m_translation.z };
	}


	// Modded skeletons sized unlike the shared ragdoll (children, giants) keep the ragdoll's joint pivots, so the
	// dynamic ragdoll pulls apart: stretched body, feet driven through the ground. While the bodies still follow the
	// animation, every joint whose two pivots no longer meet gets its own copy of the joint data with one pivot moved
	// onto the other. Needs the world write lock, before the bodies turn dynamic. Returns false if a joint that does
	// not meet could not be fitted.
	bool FitJointsToSkeleton(RE::Actor* a_actor, RE::hknpBSWorld* a_world)
	{
		RagdollMap map{};
		if (!FindRagdollMap(a_actor, a_world, map)) return true;
		auto* table = reinterpret_cast<std::uint8_t*>(a_world->m_constraintManager.constraintBuffer);
		const auto capacity = a_world->m_constraintManager.capacity;
		if (!table || capacity == 0 || capacity > 0x100000) return true;
		// hkpRagdollConstraintData 0x1A0 / hkpLimitedHingeConstraintData 0x130 bytes: transforms atom at +0x20,
		// pivot A at +0x60, pivot B at +0xA0, both in their body's local space (isValid at 0x1419303E0 / 0x14192AD80).
		static const auto ragdollType = RE::VTABLE::hkpRagdollConstraintData[0].address();
		static const auto hingeType = RE::VTABLE::hkpLimitedHingeConstraintData[0].address();
		auto& bodies = a_world->m_bodyManager.m_bodies;
		const auto toWorld = [](const RE::hknpBody& a_body, Vec3 a_local) {
			return Add(Rotate(RotationFrom(a_body.m_transform.m_rotation), a_local), BodyPosition(a_body));
		};
		const auto toLocal = [](const RE::hknpBody& a_body, Vec3 a_world) {
			return Rotate(Transpose(RotationFrom(a_body.m_transform.m_rotation)), Sub(a_world, BodyPosition(a_body)));
		};
		std::int32_t fitted = 0, failed = 0;
		float worst = 0.0f, skipped = 0.0f;
		const bool humanoid = IsHumanoid(a_actor);
		for (std::uint32_t i = 0; i < map.constraintCount; ++i) {
			const auto id = map.constraintIDs[i];
			if (id >= capacity) continue;
			auto* entry = table + static_cast<std::size_t>(id) * 0x38;
			std::uint32_t body0 = 0, body1 = 0, entryID = 0;
			std::uintptr_t data = 0;
			std::memcpy(&body0, entry, 4);
			std::memcpy(&body1, entry + 4, 4);
			std::memcpy(&data, entry + 8, 8);
			std::memcpy(&entryID, entry + 0x10, 4);
			if (entryID != id || data < 0x10000) continue;
			auto i0 = FindWorldBody(a_world, body0);
			auto i1 = FindWorldBody(a_world, body1);
			if (i0 < 0 || i1 < 0) continue;
			const auto* pivots = reinterpret_cast<const float*>(data);
			const Vec3 a{ pivots[0x60 / 4], pivots[0x64 / 4], pivots[0x68 / 4] };
			const Vec3 b{ pivots[0xA0 / 4], pivots[0xA4 / 4], pivots[0xA8 / 4] };
			const float span = std::max(Length(a), Length(b));
			float gap = Length(Sub(toWorld(bodies[i0], a), toWorld(bodies[i1], b)));
			// The entry's body order is assumed to be the data's A/B (not verified): take whichever order fits better.
			if (const float swapped = Length(Sub(toWorld(bodies[i1], a), toWorld(bodies[i0], b))); swapped < gap) {
				std::swap(i0, i1);
				gap = swapped;
			}
			// Adults (and an already fitted copy) meet within keyframe lag; child-sized skeletons were ~12 %.
			if (span < 1.0e-3f || gap < 0.08f * span) continue;
			// Fitting is for resized human skeletons: creature joints are never rewritten (the ant's fitted joints
			// stayed on the corpse and it spun forever). Small creature gaps (fog crawler) are left as they are;
			// a large one (ant: 131%) means the ragdoll does not match: no reactions.
			// shortcut: 50% limit is a guess between those two; adjust from the "creature joints" log line.
			if (!humanoid) {
				skipped = std::max(skipped, gap / span);
				if (gap > 0.5f * span) ++failed;
				continue;
			}
			const auto worldA = toWorld(bodies[i0], a);
			const auto worldB = toWorld(bodies[i1], b);
			const auto type = *reinterpret_cast<const std::uintptr_t*>(data);
			const std::int32_t size = type == ragdollType ? 0x1A0 : type == hingeType ? 0x130 : 0;
			auto* router = size ? RE::hkMemoryRouter::GetInstancePtr() : nullptr;
			auto* heap = router ? router->heap : nullptr;
			auto* clone = heap ? static_cast<std::uint8_t*>(heap->BlockAlloc(size)) : nullptr;
			if (!clone) {
				++failed;
				continue;
			}
			std::memcpy(clone, reinterpret_cast<const void*>(data), size);
			const std::uint32_t sizeAndRefs = (static_cast<std::uint32_t>(size) << 16) | 1;  // owned by this constraint
			std::memcpy(clone + 8, &sizeAndRefs, 4);
			// Keep the shorter pivot (at a body origin: the child bone's head), move the other onto it.
			const bool keepA = Length(a) <= Length(b);
			const auto moved = keepA ? toLocal(bodies[i1], worldA) : toLocal(bodies[i0], worldB);
			std::memcpy(clone + (keepA ? 0xA0 : 0x60), &moved, sizeof(moved));
			// Swap the entry (and anything else in it pointing into the data, e.g. atoms) to the copy. The shared
			// original keeps the reference this constraint held: it is never freed, but other actors may use it.
			for (std::size_t offset = 8; offset + 8 <= 0x38; offset += 8) {
				std::uintptr_t pointer = 0;
				std::memcpy(&pointer, entry + offset, 8);
				if (pointer < data || pointer >= data + size) continue;
				pointer = pointer - data + reinterpret_cast<std::uintptr_t>(clone);
				std::memcpy(entry + offset, &pointer, 8);
			}
			++fitted;
			worst = std::max(worst, gap / span);
		}
		if (fitted || failed)
			REX::INFO("DynamicRagdollSystem: {:08X} ragdoll joints fitted to its skeleton: {} (worst gap {:.0f}% of the pivot), {} not fitted"sv,
				a_actor->GetFormID(), fitted, worst * 100.0f, failed);
		if (skipped > 0.0f)
			REX::INFO("DynamicRagdollSystem: {:08X} creature joints left unfitted, worst gap {:.0f}% of the pivot{}"sv,
				a_actor->GetFormID(), skipped * 100.0f, failed ? " (over 50%: no reactions)" : "");
		return failed == 0;
	}


	bool ReadHkArray(std::uintptr_t a_object, std::size_t a_offset, std::uint32_t a_max, HkArrayView& a_out)
	{
		HkArrayView first{}, second{};
		if (!ReadField(a_object, a_offset, first) || !ReadField(a_object, a_offset, second) ||
			std::memcmp(&first, &second, sizeof(first)) != 0 || first.size < 0 ||
			static_cast<std::uint32_t>(first.size) > a_max ||
			(static_cast<std::uint32_t>(first.capacityAndFlags) & 0x3FFFFFFFu) < static_cast<std::uint32_t>(first.size) ||
			(first.size && (first.data < 0x10000 || first.data >= 0x800000000000ULL))) {
			return false;
		}
		a_out = first;
		return true;
	}


	bool FindRagdollMap(RE::Actor* a_actor, RE::hknpBSWorld* a_world, RagdollMap& a_out)
	{
		if (!a_actor || !a_actor->currentProcess || !a_actor->currentProcess->middleHigh) {
			return false;
		}
		auto* manager = a_actor->currentProcess->middleHigh->animationGraphManager.get();
		if (!manager || manager->graph.empty() || manager->graph.size() > 8) {
			return false;
		}

		std::uintptr_t ragdoll = 0;
		for (const auto& graphRef : manager->graph) {
			auto* graph = graphRef.get();
			if (!graph) {
				continue;
			}
			const auto graphAddress = reinterpret_cast<std::uintptr_t>(graph);
			std::uintptr_t driver = 0, interface = 0, driverInterface = 0, candidate = 0, world = 0;
			if (!ReadField(graphAddress, 0x208, driver) || !ReadField(graphAddress, 0x210, interface) ||
				!driver || !interface || !ReadField(driver, 0x70, driverInterface) || driverInterface != interface ||
				!ReadField(interface, 0x18, candidate) || !candidate ||
				!ReadField(candidate, 0x18, world) || world != reinterpret_cast<std::uintptr_t>(a_world)) {
				continue;
			}
			if (ragdoll && ragdoll != candidate) {
				return false;
			}
			ragdoll = candidate;
		}
		if (!ragdoll) {
			return false;
		}

		std::uintptr_t skeleton = 0;
		HkArrayView bodyIDs{}, constraintIDs{}, boneToBody{}, parents{}, bones{};
		if (!ReadField(ragdoll, 0x40, skeleton) || !skeleton ||
			!ReadHkArray(ragdoll, 0x20, kMaxRagdollBones, bodyIDs) ||
			!ReadHkArray(ragdoll, 0x30, kMaxRagdollBones, constraintIDs) ||
			!ReadHkArray(ragdoll, 0x48, kMaxRagdollBones, boneToBody) ||
			!ReadHkArray(skeleton, 0x18, kMaxRagdollBones, parents) ||
			!ReadHkArray(skeleton, 0x28, kMaxRagdollBones, bones) ||
			bodyIDs.size == 0 || boneToBody.size != parents.size || parents.size != bones.size) {
			return false;
		}

		a_out = {};
		a_out.ragdoll = ragdoll;
		a_out.bodyCount = static_cast<std::uint32_t>(bodyIDs.size);
		a_out.constraintCount = static_cast<std::uint32_t>(constraintIDs.size);
		a_out.boneCount = static_cast<std::uint32_t>(bones.size);
		if (!SafeRead(bodyIDs.data, a_out.bodyIDs.data(), bodyIDs.size * sizeof(std::uint32_t)) ||
			!SafeRead(constraintIDs.data, a_out.constraintIDs.data(), constraintIDs.size * sizeof(std::uint32_t)) ||
			!SafeRead(boneToBody.data, a_out.boneToBody.data(), boneToBody.size * sizeof(std::int32_t)) ||
			!SafeRead(parents.data, a_out.parents.data(), parents.size * sizeof(std::int16_t))) {
			return false;
		}
		for (std::uint32_t i = 0; i < a_out.boneCount; ++i) {
			const auto mapped = a_out.boneToBody[i];
			const auto parent = a_out.parents[i];
			if (mapped < -1 || mapped >= static_cast<std::int32_t>(a_out.bodyCount) ||
				parent < -1 || parent >= static_cast<std::int16_t>(a_out.boneCount) || parent == static_cast<std::int16_t>(i)) return false;
			std::int32_t cursor = static_cast<std::int32_t>(i);
			for (std::uint32_t depth = 0; cursor >= 0; ++depth) {
				if (depth > a_out.boneCount) return false;
				cursor = a_out.parents[static_cast<std::uint32_t>(cursor)];
			}
		}
		for (std::uint32_t i = 0; i < a_out.bodyCount; ++i) {
			if (a_out.bodyIDs[i] >= 0x7FFFFFFFu) continue;
			for (std::uint32_t j = 0; j < i; ++j)
				if (a_out.bodyIDs[j] < 0x7FFFFFFFu && a_out.bodyIDs[i] == a_out.bodyIDs[j]) return false;
		}
		for (std::uint32_t i = 0; i < a_out.constraintCount; ++i) {
			if (a_out.constraintIDs[i] >= 0x7FFFFFFFu) continue;
			for (std::uint32_t j = 0; j < i; ++j)
				if (a_out.constraintIDs[j] < 0x7FFFFFFFu && a_out.constraintIDs[i] == a_out.constraintIDs[j]) return false;
		}

		// Recheck all array descriptors after copying their payloads to reject a changed ragdoll map.
		HkArrayView bodyIDs2{}, constraintIDs2{}, boneToBody2{}, parents2{}, bones2{};
		std::array<std::uint32_t, kMaxRagdollBones> bodyIDsCopy{}, constraintIDsCopy{};
		std::array<std::int32_t, kMaxRagdollBones> boneToBodyCopy{};
		std::array<std::int16_t, kMaxRagdollBones> parentsCopy{};
		return ReadHkArray(ragdoll, 0x20, kMaxRagdollBones, bodyIDs2) &&
			ReadHkArray(ragdoll, 0x30, kMaxRagdollBones, constraintIDs2) &&
			ReadHkArray(ragdoll, 0x48, kMaxRagdollBones, boneToBody2) &&
			ReadHkArray(skeleton, 0x18, kMaxRagdollBones, parents2) &&
			ReadHkArray(skeleton, 0x28, kMaxRagdollBones, bones2) &&
			SafeRead(bodyIDs2.data, bodyIDsCopy.data(), bodyIDs2.size * sizeof(std::uint32_t)) &&
			SafeRead(constraintIDs2.data, constraintIDsCopy.data(), constraintIDs2.size * sizeof(std::uint32_t)) &&
			SafeRead(boneToBody2.data, boneToBodyCopy.data(), boneToBody2.size * sizeof(std::int32_t)) &&
			SafeRead(parents2.data, parentsCopy.data(), parents2.size * sizeof(std::int16_t)) &&
			std::memcmp(&bodyIDs, &bodyIDs2, sizeof(bodyIDs)) == 0 &&
			std::memcmp(&constraintIDs, &constraintIDs2, sizeof(constraintIDs)) == 0 &&
			std::memcmp(&boneToBody, &boneToBody2, sizeof(boneToBody)) == 0 &&
			std::memcmp(&parents, &parents2, sizeof(parents)) == 0 &&
			std::memcmp(&bones, &bones2, sizeof(bones)) == 0 &&
			std::equal(bodyIDsCopy.begin(), bodyIDsCopy.begin() + a_out.bodyCount, a_out.bodyIDs.begin()) &&
			std::equal(constraintIDsCopy.begin(), constraintIDsCopy.begin() + a_out.constraintCount, a_out.constraintIDs.begin()) &&
			std::equal(boneToBodyCopy.begin(), boneToBodyCopy.begin() + a_out.boneCount, a_out.boneToBody.begin()) &&
			std::equal(parentsCopy.begin(), parentsCopy.begin() + a_out.boneCount, a_out.parents.begin());
	}


	std::int32_t FindWorldBody(RE::hknpBSWorld* a_world, std::uint32_t a_id)
	{
		auto& bodies = a_world->m_bodyManager.m_bodies;
		for (std::int32_t i = 0; i < bodies.size(); ++i) {
			if (bodies[i].m_id.m_value == a_id) {
				return i;
			}
		}
		return -1;
	}


	std::int32_t FindJointConstraint(RE::hknpBSWorld* a_world, const RagdollMap& a_map,
		std::uint32_t a_body, std::uint32_t a_parent)
	{
		auto* table = a_world->m_constraintManager.constraintBuffer;
		const auto capacity = a_world->m_constraintManager.capacity;
		if (!table || capacity == 0 || capacity > 0x100000) {
			return -1;
		}
		for (std::uint32_t i = 0; i < a_map.constraintCount; ++i) {
			const auto id = a_map.constraintIDs[i];
			if (id >= capacity) {
				continue;
			}
			std::array<std::uint8_t, 0x38> raw{};
			const auto address = reinterpret_cast<std::uintptr_t>(table) + static_cast<std::uintptr_t>(id) * raw.size();
			if (!SafeRead(address, raw.data(), raw.size())) {
				return -1;
			}
			std::uint32_t body0 = 0, body1 = 0, entryID = 0;
			std::uintptr_t data = 0;
			std::memcpy(&body0, raw.data(), 4);
			std::memcpy(&body1, raw.data() + 4, 4);
			std::memcpy(&data, raw.data() + 8, 8);
			std::memcpy(&entryID, raw.data() + 0x10, 4);
			if (entryID == id && data >= 0x10000 && !(raw[0x16] & 4) &&
				((body0 == a_body && body1 == a_parent) || (body1 == a_body && body0 == a_parent))) {
				return static_cast<std::int32_t>(id);
			}
		}
		return -1;
	}


	void ScanBodies(Stumble& a_stumble, RE::Actor* a_actor, RE::hknpBSWorld* a_world)
	{
		a_stumble.bodies.clear();
		a_stumble.joints.clear();

		auto* actor3D = a_actor->Get3D();
		if (!actor3D) {
			return;
		}

		RagdollMap map{};
		if (!FindRagdollMap(a_actor, a_world, map)) {
			REX::INFO("DynamicRagdollSystem: ragdoll map unavailable"sv);
			return;
		}

		auto& bodies = a_world->m_bodyManager.m_bodies;
		for (std::int32_t bone = 0; bone < static_cast<std::int32_t>(map.boneCount); ++bone) {
			const auto bodyIndex = map.boneToBody[bone];
			if (bodyIndex < 0 || static_cast<std::uint32_t>(bodyIndex) >= map.bodyCount) {
				continue;
			}
			const auto bodyId = map.bodyIDs[bodyIndex];
			if (bodyId >= 0x7FFFFFFFu) {
				continue;
			}
			const auto worldIndex = FindWorldBody(a_world, bodyId);
			if (worldIndex < 0) {
				continue;
			}
			auto& body = bodies[worldIndex];
			if (!body.m_shape || !body.m_flags.all(RE::hknpCollisionFlagsEnum::kIsDynamic)) {
				continue;
			}

			auto* obj = RE::bhkUtilFunctions::GetAVObjectFromBodyID(a_world, body.m_id);
			if (!obj) {
				continue;
			}
			const auto name = obj->GetName();
			if (name.contains("Weapon"sv) || name.contains("P-Mag"sv) || name.contains("Receiver"sv) ||
				name.contains("Grip"sv) || name.contains("Scope"sv) || name.contains("ShellCasing"sv)) {
				continue;
			}

			const auto parentBone = map.parents[bone];
			RE::hknpBodyId parentId{};
			std::int32_t parentIndex = -1;
			std::int32_t constraintId = -1;
			if (parentBone >= 0 && static_cast<std::uint32_t>(parentBone) < map.boneCount) {
				const auto parentBodyIndex = map.boneToBody[parentBone];
				if (parentBodyIndex >= 0 && static_cast<std::uint32_t>(parentBodyIndex) < map.bodyCount) {
					parentId.m_value = map.bodyIDs[parentBodyIndex];
					parentIndex = FindWorldBody(a_world, parentId.m_value);
					if (parentIndex >= 0 && bodies[parentIndex].m_flags.all(RE::hknpCollisionFlagsEnum::kIsDynamic)) {
						constraintId = FindJointConstraint(a_world, map, bodyId, parentId.m_value);
					}
				}
			}

			const bool pelvis = name.contains("Pelvis"sv) || name.contains("pelvis"sv);
			const bool spine = name.contains("SPINE"sv) || name.contains("Spine"sv) || name.contains("Chest"sv);
			const bool leg = name.contains("Leg"sv) || name.contains("Thigh"sv) || name.contains("Calf"sv) || name.contains("Foot"sv);
			std::int32_t priority = 99;
			if (name.contains("SPINE1"sv)) priority = 0;
			else if (name.contains("SPINE2"sv)) priority = 1;
			else if (name.contains("Chest"sv)) priority = 2;
			else if (name.contains("Pelvis"sv) || name.contains("pelvis"sv)) priority = 0;

			const bool foot = name.contains("Foot"sv) || name.contains("foot"sv);
			const std::string_view nameView{ name.c_str() };
			const char side = nameView.starts_with("L"sv) ? 'L' : nameView.starts_with("R"sv) ? 'R' : 0;
			const bool hand = name.contains("Hand"sv) || name.contains("hand"sv);
			const bool arm = hand || name.contains("Arm"sv) || name.contains("arm"sv);
			const bool knee = name.contains("Calf"sv) || name.contains("calf"sv);
			const bool head = name.contains("Head"sv) || name.contains("HEAD"sv) || name.contains("Neck"sv) || name.contains("NECK"sv);
			const BodyRef ref{ worldIndex, body.m_id, parentId, bone, parentIndex, constraintId, pelvis, spine, leg, priority, foot, side, hand, arm, knee, head };
			a_stumble.bodies.push_back(ref);
			if (constraintId >= 0 && parentIndex >= 0 && (spine || leg || arm)) {  // arms: dying twitches only
				a_stumble.joints.push_back(ref);
			}
		}

		if (!a_stumble.bodies.empty()) {
			const auto& first = a_world->m_bodyManager.m_bodies[a_stumble.bodies.front().index];
			REX::INFO("DynamicRagdollSystem: ragdoll body collision layer {} (filter {:08X})"sv,
				first.m_collisionFilterInfo & 0x7F, first.m_collisionFilterInfo);
			// Humanoids only: the part table is shared by every ragdoll, and creature "Arm"/"Hand" bones added pairs
			// that made other ragdolls' bodies collide with themselves.
			if (IsHumanoid(a_actor)) LearnArmTorsoParts(a_stumble, a_world);
		}
		REX::INFO("DynamicRagdollSystem: ragdoll map bones {} bodies {} dynamic {} joints {}"sv,
			map.boneCount, map.bodyCount, a_stumble.bodies.size(), a_stumble.joints.size());
		for (const auto& body : a_stumble.joints) {
			if (body.spine || body.pelvis) {
				auto* obj = RE::bhkUtilFunctions::GetAVObjectFromBodyID(a_world, body.id);
				REX::INFO("joint bone {} '{}' parent body {} constraint {}"sv,
					body.bone, obj ? obj->GetName().c_str() : "?", body.parentId.m_value, body.constraintId);
			}
		}
	}


	// --- v0.1.x joint/balance helpers (restored for the death stumble) ---
	bool Solve3(float matrix[3][3], Vec3 rhs, Vec3& result)
	{
		double a[3][4] = {
			{ matrix[0][0], matrix[0][1], matrix[0][2], rhs.x },
			{ matrix[1][0], matrix[1][1], matrix[1][2], rhs.y },
			{ matrix[2][0], matrix[2][1], matrix[2][2], rhs.z }
		};
		for (int col = 0; col < 3; ++col) {
			int pivot = col;
			for (int row = col + 1; row < 3; ++row) {
				if (std::abs(a[row][col]) > std::abs(a[pivot][col])) {
					pivot = row;
				}
			}
			if (!std::isfinite(a[pivot][col]) || std::abs(a[pivot][col]) < 1.0e-10) {
				return false;
			}
			if (pivot != col) {
				for (int j = col; j < 4; ++j) {
					std::swap(a[pivot][j], a[col][j]);
				}
			}
			const auto divisor = a[col][col];
			for (int j = col; j < 4; ++j) {
				a[col][j] /= divisor;
			}
			for (int row = 0; row < 3; ++row) {
				if (row == col) continue;
				const auto factor = a[row][col];
				for (int j = col; j < 4; ++j) {
					a[row][j] -= factor * a[col][j];
				}
			}
		}
		result = { static_cast<float>(a[0][3]), static_cast<float>(a[1][3]), static_cast<float>(a[2][3]) };
		return std::isfinite(result.x) && std::isfinite(result.y) && std::isfinite(result.z);
	}


	void AddWorldInverse(const MotionState& a_motion, float a_matrix[3][3])
	{
		const Vec3 axes[3] = {
			Rotate(a_motion.orientation, { 1, 0, 0 }),
			Rotate(a_motion.orientation, { 0, 1, 0 }),
			Rotate(a_motion.orientation, { 0, 0, 1 })
		};
		for (int k = 0; k < 3; ++k) {
			const float v[3] = { axes[k].x, axes[k].y, axes[k].z };
			for (int r = 0; r < 3; ++r) {
				for (int c = 0; c < 3; ++c) {
					a_matrix[r][c] += a_motion.inverseInertia[k] * v[r] * v[c];
				}
			}
		}
	}


	Vec3 ApplyWorldInverse(const MotionState& a_motion, Vec3 a_impulse)
	{
		const auto local = Rotate(Conjugate(a_motion.orientation), a_impulse);
		return Rotate(a_motion.orientation, {
			local.x * a_motion.inverseInertia[0],
			local.y * a_motion.inverseInertia[1],
			local.z * a_motion.inverseInertia[2]
		});
	}


	bool ConstraintCurrent(RE::hknpBSWorld* a_world, std::int32_t a_id, std::uint32_t a_body, std::uint32_t a_parent)
	{
		auto* table = a_world->m_constraintManager.constraintBuffer;
		const auto capacity = a_world->m_constraintManager.capacity;
		if (!table || a_id < 0 || static_cast<std::uint32_t>(a_id) >= capacity) return false;
		std::array<std::uint8_t, 0x38> raw{};
		const auto address = reinterpret_cast<std::uintptr_t>(table) + static_cast<std::uintptr_t>(a_id) * raw.size();
		if (!SafeRead(address, raw.data(), raw.size())) return false;
		std::uint32_t body0 = 0, body1 = 0, entryID = 0;
		std::uintptr_t data = 0;
		std::memcpy(&body0, raw.data(), 4);
		std::memcpy(&body1, raw.data() + 4, 4);
		std::memcpy(&data, raw.data() + 8, 8);
		std::memcpy(&entryID, raw.data() + 0x10, 4);
		return entryID == static_cast<std::uint32_t>(a_id) && data >= 0x10000 && !(raw[0x16] & 4) &&
			((body0 == a_body && body1 == a_parent) || (body1 == a_body && body0 == a_parent));
	}


	bool SolveJointImpulse(const MotionState& a_child, const MotionState& a_parent, Vec3 a_desiredDelta, Vec3& a_impulse,
		float a_maxImpulse)
	{
		float matrix[3][3]{};
		AddWorldInverse(a_child, matrix);
		AddWorldInverse(a_parent, matrix);
		if (!Solve3(matrix, a_desiredDelta, a_impulse)) return false;
		const float impulseNorm = Length(a_impulse);
		if (!std::isfinite(impulseNorm) || impulseNorm < 1.0e-6f) return false;
		if (impulseNorm > a_maxImpulse) a_impulse = Scale(a_impulse, a_maxImpulse / impulseNorm);
		const auto childDelta = ApplyWorldInverse(a_child, a_impulse);
		const auto parentDelta = ApplyWorldInverse(a_parent, Scale(a_impulse, -1.0f));
		const auto relativeDelta = Sub(childDelta, parentDelta);
		return std::isfinite(relativeDelta.x) && std::isfinite(relativeDelta.y) && std::isfinite(relativeDelta.z) &&
			Dot(relativeDelta, a_desiredDelta) > 0.0f;
	}


	// Dismemberment removes a limb's joint constraint. That limb and everything below it is then no longer part of
	// the body: left in, the cut leg counted as a foot and in the centre of mass, and a grip on it kept pulling
	// (floating). Only limbs: a passing constraint glitch on the spine must not drop the whole torso.
	void UpdateDetached(Stumble& a_stumble, RE::hknpBSWorld* a_world)
	{
		for (bool changed = true; changed;) {
			changed = false;
			for (const auto& body : a_stumble.bodies) {
				if (body.constraintId < 0 || body.spine || body.pelvis || a_stumble.Detached(body.id.m_value)) continue;
				if (!a_stumble.Detached(body.parentId.m_value) &&
					ConstraintCurrent(a_world, body.constraintId, body.id.m_value, body.parentId.m_value)) continue;
				a_stumble.detached.push_back(body.id.m_value);
				changed = true;
				REX::INFO("DynamicRagdollSystem: {:08X} body {} detached (dismembered limb); ignored from now on"sv, a_stumble.formID,
					body.id.m_value);
			}
		}
	}


	bool ApplyJointImpulse(RE::hknpBSWorld* a_world, const BodyRef& a_childRef, const BodyRef& a_parentRef, Vec3 a_desiredDelta,
		float a_maxImpulse)
	{
		if (!g_angularImpulse || a_childRef.index < 0 || a_parentRef.index < 0 ||
			a_childRef.index >= a_world->m_bodyManager.m_bodies.size() ||
			a_parentRef.index >= a_world->m_bodyManager.m_bodies.size()) return false;
		auto& child = a_world->m_bodyManager.m_bodies[a_childRef.index];
		auto& parent = a_world->m_bodyManager.m_bodies[a_parentRef.index];
		if (child.m_id.m_value != a_childRef.id.m_value || parent.m_id.m_value != a_parentRef.id.m_value ||
			!ConstraintCurrent(a_world, a_childRef.constraintId, child.m_id.m_value, parent.m_id.m_value)) return false;
		MotionState childMotion{}, parentMotion{};
		if (!ReadMotionState(a_world, child, childMotion) || !ReadMotionState(a_world, parent, parentMotion)) return false;
		Vec3 impulse{};
		if (!SolveJointImpulse(childMotion, parentMotion, a_desiredDelta, impulse, a_maxImpulse)) return false;
		const auto beforeRelative = Sub(Rotate(childMotion.orientation, childMotion.angular),
			Rotate(parentMotion.orientation, parentMotion.angular));
		RE::hkVector4f childImpulse{ impulse.x, impulse.y, impulse.z, 0.0f };
		RE::hkVector4f parentImpulse{ -impulse.x, -impulse.y, -impulse.z, 0.0f };
		g_angularImpulse(a_world, child.m_id.m_value, &childImpulse);
		g_angularImpulse(a_world, parent.m_id.m_value, &parentImpulse);
		MotionState childAfter{}, parentAfter{};
		if (!ReadMotionState(a_world, child, childAfter) || !ReadMotionState(a_world, parent, parentAfter)) return false;
		const auto afterRelative = Sub(Rotate(childAfter.orientation, childAfter.angular),
			Rotate(parentAfter.orientation, parentAfter.angular));
		const auto observed = Sub(afterRelative, beforeRelative);
		return std::isfinite(observed.x) && std::isfinite(observed.y) && std::isfinite(observed.z) &&
			Length(observed) >= 1.0e-5f && Dot(observed, a_desiredDelta) > 0.0f;
	}



	void ResolveImpulseFunction()
	{
		// Exact bytes of the two hknpBSWorld impulse wrappers: OG 35 bytes, AE 40 (tail-jumps to the activate call).
		static constexpr std::uint8_t linearOG[] = {
			0x89,0x54,0x24,0x10,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xD9,0xE8,0xCF,0x6C,0x74,0xFF,
			0x8B,0x54,0x24,0x38,0x48,0x8B,0xCB,0xE8,0x03,0x08,0x00,0x00,0x48,0x83,0xC4,0x20,0x5B,0xC3
		};
		static constexpr std::uint8_t angularOG[] = {
			0x89,0x54,0x24,0x10,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xD9,0xE8,0x9F,0x6D,0x74,0xFF,
			0x8B,0x54,0x24,0x38,0x48,0x8B,0xCB,0xE8,0xD3,0x07,0x00,0x00,0x48,0x83,0xC4,0x20,0x5B,0xC3
		};
		static constexpr std::uint8_t linearAE[] = {
			0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x8B,0xDA,0x48,0x8B,0xF9,0xE8,0x4C,0xCD,0x9E,0xFF,
			0x8B,0xD3,0x48,0x8B,0xCF,0x48,0x8B,0x5C,0x24,0x30,0x48,0x83,0xC4,0x20,0x5F,0xE9,0xC8,0x07,0x00,0x00
		};
		static constexpr std::uint8_t angularAE[] = {
			0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x8B,0xDA,0x48,0x8B,0xF9,0xE8,0x1C,0xCE,0x9E,0xFF,
			0x8B,0xD3,0x48,0x8B,0xCF,0x48,0x8B,0x5C,0x24,0x30,0x48,0x83,0xC4,0x20,0x5F,0xE9,0x98,0x07,0x00,0x00
		};
		const bool ae = REX::FModule::IsRuntimeAE();
		const auto matches = [](std::uintptr_t a_address, const std::uint8_t* a_expected, std::size_t a_size) {
			std::uint8_t bytes[sizeof(linearAE)]{};
			return a_size <= sizeof(bytes) && SafeRead(a_address, bytes, a_size) && std::memcmp(bytes, a_expected, a_size) == 0;
		};
		const auto linear = REL::ID{ 488591, 0, 2721630 }.address();
		if (!(ae ? matches(linear, linearAE, sizeof(linearAE)) : matches(linear, linearOG, sizeof(linearOG)))) {
			REX::INFO("DynamicRagdollSystem: impulse API fingerprint mismatch; hit push disabled"sv);
			return;
		}
		g_linearImpulse = reinterpret_cast<LinearImpulse>(linear);
		const auto angular = REL::ID{ 222853, 0, 2721631 }.address();
		if (ae ? matches(angular, angularAE, sizeof(angularAE)) : matches(angular, angularOG, sizeof(angularOG))) {
			g_angularImpulse = reinterpret_cast<AngularImpulse>(angular);
		} else {
			REX::INFO("DynamicRagdollSystem: angular impulse fingerprint mismatch; death stumble without joint servo"sv);
		}
	}


	// bhkUtilFunctions::SetMotionType over an object tree; the engine's RagdollStart handler calls it
	// as (root, kDynamic, 1, 1, 1).
	void SetTreeMotionType(RE::NiAVObject* a_root, RE::hknpMotionPropertiesId::Preset a_type)
	{
		using func_t = bool (*)(RE::NiAVObject*, RE::hknpMotionPropertiesId::Preset, bool, bool, bool);
		static REL::Relocation<func_t> func{ REL::ID{ 357289, 0, 2277723 } };
		func(a_root, a_type, true, true, true);
	}


	// Sets the collision layer (low 7 bits of the filter info) on every collision object under a_root, as the
	// knockdown does with layer 8 (ID 715635, 0x141D8DA30).
	void SetTreeCollisionLayer(RE::NiAVObject* a_root, RE::COL_LAYER a_layer)
	{
		using func_t = void (*)(RE::NiAVObject*, std::uint32_t);
		static REL::Relocation<func_t> func{ REL::ID{ 715635, 0, 2278105 } };
		func(a_root, static_cast<std::uint32_t>(a_layer));
	}

	// bhkCollisionFilter (pointer global ID 469495, 0x1458E0AD8), pair check ID 147831 (0x141D88E60): u64 collide mask
	// per layer at +0x1A0 (bit n = collides with layer n). Two bodies of one ragdoll (same group) on layers 8/32/33
	// also need, in the u32 per-part table at +0x20, bit partB in entry partA (part = filter info bits 8-12).
	// bRagdollCollideWithNPCs: 8 <-> 8 and 32 <-> 8 (ragdolls vs living NPC bodies). bRagdollSelfCollision: 8 -> 8,
	// 32 -> 32 and the forearm/hand <-> torso part pairs learnt by LearnArmTorsoParts. Recomputed from the engine's
	// original bits every update, so either toggle can go off again (hot reload).
	namespace
	{
		std::array<std::uint32_t, 32> g_learntParts{};
	}

	void UpdateCollisionFilter()
	{
		static bool                          saved = false;
		static std::uint64_t                 orig8 = 0, orig32 = 0;
		static std::array<std::uint32_t, 32> origParts{};
		static std::uint64_t                 last8 = 0, last32 = 0;
		static REL::Relocation<std::uintptr_t*> global{ REL::ID{ 469495, 0, 4797438 } };
		const auto filter = *global;
		if (!filter) return;  // not created yet: retried next update
		auto* masks = reinterpret_cast<std::uint64_t*>(filter + 0x1A0);
		auto* parts = reinterpret_cast<std::uint32_t*>(filter + 0x20);
		if (!saved) {
			saved = true;
			orig8 = last8 = masks[8];
			orig32 = last32 = masks[32];
			std::copy_n(parts, 32, origParts.begin());
		}
		constexpr std::uint64_t kBit8 = 1ull << 8, kBit32 = 1ull << 32;
		const std::uint64_t want8 = orig8 | (kRagdollCollision ? kBit8 | kBit32 : 0) | (kSelfCollision ? kBit8 : 0);
		const std::uint64_t want32 = orig32 | (kRagdollCollision ? kBit8 : 0) | (kSelfCollision ? kBit32 : 0);
		masks[8] = want8;
		masks[32] = want32;
		for (int i = 0; i < 32; ++i) parts[i] = origParts[i] | (kSelfCollision ? g_learntParts[i] : 0);
		if (want8 != last8 || want32 != last32) {
			// layer 33 (hit reactions) is not changed, but shares the part table if its mask has bit 33
			REX::INFO("DynamicRagdollSystem: collision filter: NPC contact {} self {} (layer 8 mask {:016X} -> {:016X}, layer 32 mask {:016X} -> {:016X}, layer 33 mask {:016X})"sv,
				kRagdollCollision, kSelfCollision, last8, want8, last32, want32, masks[33]);
			last8 = want8;
			last32 = want32;
		}
	}

	// Part numbers of this ragdoll's forearms/hands (arm bodies whose parent is an arm body; not the upper arm, which
	// sits against the shoulder) and trunk bodies; each arm/trunk pair is added to g_learntParts.
	void LearnArmTorsoParts(const Stumble& a_stumble, RE::hknpBSWorld* a_world)
	{
		auto& bodies = a_world->m_bodyManager.m_bodies;
		const auto info = [&](const BodyRef& a_b) -> std::int64_t {
			if (a_b.index < 0 || a_b.index >= bodies.size() || bodies[a_b.index].m_id.m_value != a_b.id.m_value) return -1;
			return bodies[a_b.index].m_collisionFilterInfo;
		};
		std::uint32_t added = 0;
		for (const auto& arm : a_stumble.bodies) {
			if (!arm.arm || std::none_of(a_stumble.bodies.begin(), a_stumble.bodies.end(),
								[&](const BodyRef& a_p) { return a_p.arm && a_p.id.m_value == arm.parentId.m_value; })) continue;
			const auto armInfo = info(arm);
			if (armInfo < 0) continue;
			for (const auto& trunk : a_stumble.bodies) {
				if (!trunk.spine && !trunk.pelvis) continue;
				const auto trunkInfo = info(trunk);
				if (trunkInfo < 0) continue;
				const auto a = static_cast<std::uint32_t>(armInfo >> 8) & 0x1F, t = static_cast<std::uint32_t>(trunkInfo >> 8) & 0x1F;
				if (a == t) continue;
				if (!(g_learntParts[a] & (1u << t))) ++added;
				g_learntParts[a] |= 1u << t;
				g_learntParts[t] |= 1u << a;
				if (armInfo & trunkInfo & 0x8000) {  // both bit 15: the engine uses bone adjacency instead of the table
					REX::INFO("DynamicRagdollSystem: self collision: arm/trunk bodies use adjacency rule (filter {:08X} / {:08X})"sv,
						static_cast<std::uint32_t>(armInfo), static_cast<std::uint32_t>(trunkInfo));
				}
			}
		}
		if (added > 0) REX::INFO("DynamicRagdollSystem: self collision: {} new arm/trunk part pairs"sv, added);
	}

	// Lowest (pelvis / SPINE1) and highest (chest) trunk bodies among the samples; same if only one.
	std::pair<const DeathSample*, const DeathSample*> TrunkEnds(const std::vector<DeathSample>& a_samples)
	{
		const DeathSample* low = nullptr;
		const DeathSample* high = nullptr;
		for (const auto& sample : a_samples) {
			if (!sample.ref->spine && !sample.ref->pelvis) continue;
			if (!low || sample.ref->priority < low->ref->priority) low = &sample;
			if (!high || sample.ref->priority > high->ref->priority) high = &sample;
		}
		return { low, high };
	}

	// Per joint: a random axis and rhythm (1.2–2.2 Hz, random phase) for the flail. Rebuilt when the joints change.
	void InitFlail(Stumble& a_stumble)
	{
		if (a_stumble.flailAxes.size() == a_stumble.joints.size()) return;
		static std::mt19937 rng{ std::random_device{}() };
		std::uniform_real_distribution<float> unit{ 0.0f, 1.0f };
		a_stumble.flailAxes.clear();
		a_stumble.flailRhythm.clear();
		for (std::size_t i = 0; i < a_stumble.joints.size(); ++i) {
			a_stumble.flailAxes.push_back(Unit(Vec3{ unit(rng) * 2.0f - 1.0f, unit(rng) * 2.0f - 1.0f, unit(rng) * 2.0f - 1.0f }));
			a_stumble.flailRhythm.push_back({ 6.2831853f * (1.2f + 1.0f * unit(rng)), 6.2831853f * unit(rng) });
		}
	}

	// Flail every arm and leg joint (except the arm on a_skipArm / leg on a_skipLeg side). a_amount 0..1: how much the
	// body is flying; a_strength: swing size. Returns the joints driven.
	std::int32_t FlailLimbs(RE::hknpBSWorld* a_world, Stumble& a_stumble, float a_time, float a_dt, float a_amount, float a_strength,
		char a_skipArm, char a_skipLeg)
	{
		InitFlail(a_stumble);
		std::int32_t flailing = 0;
		for (std::size_t i = 0; i < a_stumble.joints.size(); ++i) {
			const auto& joint = a_stumble.joints[i];
			if (!joint.arm && !joint.leg) continue;
			if ((joint.arm && a_skipArm && joint.side == a_skipArm) || (joint.leg && a_skipLeg && joint.side == a_skipLeg)) continue;
			// Strength sets the swing size (amplitude); tracking is tight so joint friction cannot flatten it.
			const float peak = kFlailArmSpeed * (joint.arm ? 1.0f : 0.6f) * a_strength;
			const float gain = std::min(1.0f, kFlailGain * a_dt) * std::min(a_amount, 1.0f);
			if (DriveFlailJoint(a_world, a_stumble, i, a_time, peak, gain, kFlailMaxTorque * a_strength * a_dt)) ++flailing;
		}
		return flailing;
	}

	// Flail one joint: its relative angular velocity tracks a vector of length a_peak that circles between the joint's
	// random axis and a perpendicular one (parent frame), so the limb loops instead of pinning at a joint limit.
	// Paired joint impulse (muscle-like). a_gain: fraction of the error corrected this step; a_cap: impulse cap.
	bool DriveFlailJoint(RE::hknpBSWorld* a_world, const Stumble& a_stumble, std::size_t a_joint, float a_time, float a_peak,
		float a_gain, float a_cap)
	{
		if (a_joint >= a_stumble.joints.size() || a_joint >= a_stumble.flailAxes.size()) return false;
		const auto& joint = a_stumble.joints[a_joint];
		if (a_stumble.Detached(joint.id.m_value)) return false;
		auto& bodies = a_world->m_bodyManager.m_bodies;
		if (joint.index < 0 || joint.index >= bodies.size() || joint.parentIndex < 0 || joint.parentIndex >= bodies.size()) return false;
		auto& child = bodies[joint.index];
		auto& parent = bodies[joint.parentIndex];
		if (child.m_id.m_value != joint.id.m_value || parent.m_id.m_value != joint.parentId.m_value) return false;
		MotionState childMotion{}, parentMotion{};
		if (!ReadMotionState(a_world, child, childMotion) || !ReadMotionState(a_world, parent, parentMotion)) return false;
		const auto relativeOmega = Sub(Rotate(childMotion.orientation, childMotion.angular),
			Rotate(parentMotion.orientation, parentMotion.angular));
		const auto [frequency, phase] = a_stumble.flailRhythm[a_joint];
		const auto& base = a_stumble.flailAxes[a_joint];
		const Vec3 helper = std::abs(base.z) < 0.9f ? Vec3{ 0.0f, 0.0f, 1.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
		const auto side = Unit(Cross(base, helper));
		const float angle = frequency * a_time + phase;
		const auto axis = Rotate(RotationFrom(parent.m_transform.m_rotation), Add(Scale(base, std::cos(angle)), Scale(side, std::sin(angle))));
		const auto desired = Scale(Sub(Scale(axis, a_peak), relativeOmega), a_gain);
		BodyRef parentRef{};
		parentRef.index = joint.parentIndex;
		parentRef.id = joint.parentId;
		return ApplyJointImpulse(a_world, joint, parentRef, desired, a_cap);
	}
}
