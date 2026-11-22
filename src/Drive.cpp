#include "Common.h"

#include <shared_mutex>

namespace DynamicRagdollSystem
{
	// Writes rigid-body ragdoll controls into the generator output before driveToPose reads it, so the
	// engine's own keyframe-hierarchy controller pulls the (dynamic) ragdoll toward the animation.
	// Track layout: count at +4, header i at +0x10 + i*0x10 {s16 capacity, s16 numData, s16 dataOffset,
	// s16 elementSize, f32 onFraction, u8 flags, u8 type}; sparse data = palette, then an s8 bone->palette map.
	bool InjectRigidBodyControls(std::uintptr_t a_outputHolder, float a_stiffness, float a_weight, float a_dt, float a_catchUp,
		float a_armStiffness, DriveState& a_state)
	{
		struct Header { std::int16_t capacity, numData, dataOffset, elementSize; float onFraction; std::uint8_t flags, type, pad[2]; };
		std::uintptr_t output = 0;
		std::int32_t   trackCount = 0;
		if (!ReadField(a_outputHolder, 0, output) || !output || !ReadField(output, 4, trackCount) || trackCount < 16) {
			if (!a_state.logged) {
				a_state.logged = true;
				REX::INFO("DynamicRagdollSystem: {:08X} generator output unusable (output {:X}, tracks {})"sv, a_state.formID, output, trackCount);
			}
			return false;
		}
		auto* rigid = reinterpret_cast<Header*>(output + 0x10 + 13 * 0x10);
		auto* powered = reinterpret_cast<Header*>(output + 0x10 + 15 * 0x10);
		if (rigid->elementSize != sizeof(RigidBodyControl) || rigid->capacity <= 0 || rigid->capacity > kMaxRagdollBones || rigid->dataOffset <= 0) {
			// Undrivable (more than kMaxRagdollBones bones): end the reaction now and skip later hits; left dynamic it sank.
			// Caller holds g_driveMutex.
			g_mismatched.insert(a_state.formID);
			if (!a_state.logged) {
				a_state.logged = true;
				REX::INFO("DynamicRagdollSystem: {:08X} unexpected rigid-body track (cap {} elem {:#x}); not driving"sv,
					a_state.formID, rigid->capacity, rigid->elementSize);
			}
			return false;
		}
		const auto makeControl = [&](float a_scale) {
			RigidBodyControl control{};
			// All gains soften on the hit: at full strength the velocity gain cancels the hit push within a few steps.
			control.hierarchyGain *= a_scale;
			control.accelerationGain *= a_scale;
			control.velocityGain *= a_scale;
			control.positionGain *= a_scale;
			control.snapGain *= a_scale;
			// These gains are fractions corrected per update. At high frame rates the engine updates more often and
			// the hit is pulled back much faster, so convert them to the same correction per second as at 30 updates/s.
			const float steps = std::clamp(a_dt * kReferenceRate, 0.05f, 4.0f);
			for (float* gain : { &control.hierarchyGain, &control.velocityGain, &control.positionGain, &control.snapGain }) {
				*gain = 1.0f - std::pow(1.0f - std::clamp(*gain, 0.0f, 0.99f), steps);
			}
			control.positionMaxLinearVelocity = a_catchUp;  // default 1.4 is below running speed
			return control;
		};
		auto* data = reinterpret_cast<std::uint8_t*>(output + rigid->dataOffset);
		const auto mapOffset = (static_cast<std::size_t>(rigid->capacity) * rigid->elementSize + 0xF) & ~std::size_t{ 0xF };
		auto* map = data + mapOffset;
		const auto control = makeControl(a_stiffness);
		std::memcpy(data, &control, sizeof(control));
		std::memset(map, 0, static_cast<std::size_t>(rigid->capacity));  // every bone uses palette entry 0 ...
		std::int16_t entries = 1;
		const auto addEntry = [&](float a_entryStiffness, const BoneMask& a_bones) {
			if (a_bones.none() || entries >= rigid->capacity) return;
			const auto entry = makeControl(a_entryStiffness);
			std::memcpy(data + static_cast<std::size_t>(entries) * rigid->elementSize, &entry, sizeof(entry));
			for (std::int32_t bone = 0; bone < rigid->capacity && bone < kMaxRagdollBones; ++bone)
				if (a_bones[bone]) map[bone] = static_cast<std::uint8_t>(entries);
			++entries;
		};
		// ... except the arms while burning: their own, much softer entry so they can flail.
		if (a_armStiffness >= 0.0f) addEntry(a_armStiffness, a_state.armBones);
		// Feet at full strength: on the soft hit the light foot bodies were pressed into the ground by the body's
		// weight. Not while burning (the burn fall needs loose legs).
		// Recovery stiffness blend (not pinned): the whole legs too; at fBashRecoverStiffness a 1.7x skeleton sagged at the knees.
		else addEntry(1.0f, a_state.recover && a_state.poses.empty() ? a_state.legBones : a_state.footBones);
		rigid->numData = entries;
		rigid->onFraction = a_weight;
		powered->onFraction = 0.0f;  // rigid-body control only
		// Track 16 = hkbWorldFromModelModeData {s16 bones[3]; s8 mode}. Its default 0 (USE_OLD) froze the
		// actor's root while the ragdoll was driven; USE_INPUT (1) keeps taking it from the animation/controller.
		if (auto* worldFromModel = reinterpret_cast<Header*>(output + 0x10 + 16 * 0x10);
			worldFromModel->elementSize == 8 && worldFromModel->capacity >= 1 && worldFromModel->dataOffset > 0) {
			reinterpret_cast<std::uint8_t*>(output + worldFromModel->dataOffset)[6] = 1;
			worldFromModel->numData = 1;
			worldFromModel->onFraction = 1.0f;
		}
		if (!a_state.logged) {
			a_state.logged = true;
			REX::INFO("DynamicRagdollSystem: {:08X} driving ragdoll (rigid-body track cap {})"sv, a_state.formID, rigid->capacity);
		}
		return true;
	}


	// driveToPose maps the animation onto the ragdoll skeleton (hkaSkeletonMapper::mapPose, mode 2), which ends by
	// giving every parentless bone flagged lockTranslation its reference-pose translation. On AE (1.11.240) humans map
	// onto 'Ragdoll_NPC COM' from skeleton.hkx, whose root has that flag: the target stayed at the standing pelvis
	// height (68.9 up), so crouched NPCs floated. OG maps onto the .nif ragdoll skeleton (root clear). See
	// RE_NOTES_AE.md. Clear it so the root follows the animation. Driver +0x68 -> +0x78 -> +0x30 mapper, +0x18 ragdoll
	// skeleton: parents +0x18 (s16), bones +0x28 (0x10 each, lockTranslation at +8), count +0x30.
	// The skeleton is shared by every human; only this mapping step was checked as a reader of the flag.
	static std::uintptr_t RagdollSkeleton(std::uintptr_t a_driver)
	{
		std::uintptr_t a = 0, b = 0, mapper = 0, skeleton = 0;
		if (!ReadField(a_driver, 0x68, a) || !a || !ReadField(a, 0x78, b) || !b || !ReadField(b, 0x30, mapper) || !mapper ||
			!ReadField(mapper, 0x18, skeleton))
			return 0;
		return skeleton;
	}

	static void UnlockRagdollRoot(std::uintptr_t a_driver, RE::TESFormID a_formID)
	{
		const std::uintptr_t skeleton = RagdollSkeleton(a_driver);
		std::uintptr_t parents = 0, bones = 0;
		std::int32_t   count = 0;
		if (!skeleton || !ReadField(skeleton, 0x18, parents) || !ReadField(skeleton, 0x28, bones) ||
			!ReadField(skeleton, 0x30, count) || !parents || !bones || count <= 0 || count > kMaxRagdollBones)
			return;
		for (std::int32_t i = 0; i < count; ++i) {
			std::int16_t parent = 0;
			std::uint8_t locked = 0;
			if (!ReadField(parents, i * 2, parent) || parent != -1 || !ReadField(bones, i * 0x10 + 8, locked) || !locked) continue;
			*reinterpret_cast<std::uint8_t*>(bones + i * 0x10 + 8) = 0;
			REX::INFO("DynamicRagdollSystem: {:08X} ragdoll root bone {} had lockTranslation (drive target pinned to the reference pose); cleared"sv,
				a_formID, i);
		}
	}


	// The recovery pose moves with the actor (its AI walks and attacks right away): rotated by its turn about it, then
	// carried to where it is. Left in place, the mesh was blended between there and the actor a metre away.
	BonePose PlaceRecoveryPose(BonePose a_pose, Vec3 a_anchor, float a_yaw, const RE::Actor* a_actor)
	{
		if (!a_actor) return a_pose;
		constexpr float kGameToPhysics = 0.0142875f;
		const auto  pos = a_actor->GetPosition();
		const float turn = -(a_actor->data.angle.z - a_yaw);  // heading is clockwise; world rotation about z is counter-clockwise
		const float c = std::cos(turn), s = std::sin(turn), sz = std::sin(0.5f * turn), w = std::cos(0.5f * turn);
		const float x = a_pose.translation[0] - a_anchor.x, y = a_pose.translation[1] - a_anchor.y;
		a_pose.translation[0] = pos.x * kGameToPhysics + c * x - s * y;
		a_pose.translation[1] = pos.y * kGameToPhysics + s * x + c * y;
		a_pose.translation[2] += pos.z * kGameToPhysics - a_anchor.z;
		const Quat q{ a_pose.rotation[0], a_pose.rotation[1], a_pose.rotation[2], a_pose.rotation[3] };  // (0,0,sz,w) * q
		a_pose.rotation[0] = w * q.x - sz * q.y;
		a_pose.rotation[1] = w * q.y + sz * q.x;
		a_pose.rotation[2] = w * q.z + sz * q.w;
		a_pose.rotation[3] = w * q.w - sz * q.z;
		return a_pose;
	}

	// Main thread (g_mutex held), once per recovery, a moment after pinning: the bodies must be where they were pinned.
	// Pinning assumes each ragdoll bone's transform is its body's; on a super mutant they ended up distorted. Any body
	// more than kPinTolerance off: pinning off for this recovery, the stiffness blend instead.
	void CheckRecoveryPins(const Stumble& a_stumble, RE::Actor* a_actor, float a_now)
	{
		constexpr float kPinTolerance = 0.1f;  // physics units (m); the actor's own step between drive calls is a few cm
		std::vector<BonePose> poses;
		Vec3  anchor{};
		float yaw = 0.0f;
		{
			std::scoped_lock lock{ g_driveMutex };
			const auto it = std::find_if(g_drives.begin(), g_drives.end(), [&](const auto& a_e) { return a_e.second.formID == a_stumble.formID; });
			if (it == g_drives.end()) return;
			const auto& state = it->second;
			if (!state.recover || state.poses.empty() || state.posesChecked || state.posesUsable != 1 || a_now - state.start < 0.1f) return;
			poses = state.poses;
			anchor = state.poseAnchor;
			yaw = state.poseYaw;
		}
		auto* world = GetWorld(a_actor);
		auto* npWorld = world ? world->m_worldNP.get() : nullptr;
		if (!npWorld) return;
		float worst = 0.0f;
		int   worstBone = -1;
		std::string detail;  // per body: pinned target vs where it is, for working out the engine's bone -> body conversion
		{
			WorldWriteLock lock{ npWorld };
			if (!lock.held) return;  // next frame
			for (const auto& pose : poses) {
				const auto ref = std::find_if(a_stumble.bodies.begin(), a_stumble.bodies.end(), [&](const BodyRef& a_b) { return a_b.id.m_value == pose.body; });
				const auto* body = ref != a_stumble.bodies.end() ? FindBody(npWorld->m_bodyManager.m_bodies, *ref) : nullptr;
				if (!body) continue;
				const auto  want = PlaceRecoveryPose(pose, anchor, yaw, a_actor);
				const auto  got = BodyPosition(*body);
				const float off = Length(Sub(got, Vec3{ want.translation[0], want.translation[1], want.translation[2] }));
				const auto  q = QuatFrom(body->m_transform.m_rotation);
				detail += std::format("\n    bone {:2} want ({:.3f},{:.3f},{:.3f} | {:.3f},{:.3f},{:.3f},{:.3f}) got ({:.3f},{:.3f},{:.3f} | {:.3f},{:.3f},{:.3f},{:.3f})",
					pose.bone, want.translation[0], want.translation[1], want.translation[2], want.rotation[0], want.rotation[1], want.rotation[2],
					want.rotation[3], got.x, got.y, got.z, q.x, q.y, q.z, q.w);
				if (off > worst) {
					worst = off;
					worstBone = pose.bone;
				}
			}
		}
		std::scoped_lock lock{ g_driveMutex };
		for (auto& [driver, state] : g_drives) {
			if (state.formID != a_stumble.formID || !state.recover) continue;
			state.posesChecked = true;
			if (worst > kPinTolerance) state.poses.clear();
		}
		const auto pos = a_actor->GetPosition();
		REX::INFO("DynamicRagdollSystem: {:08X} recovery pins: worst body {:.3f} off (bone {}){}"sv, a_stumble.formID, worst, worstBone,
			worst > kPinTolerance ? std::format("; pinning off, stiffness blend instead. actor ({:.3f},{:.3f},{:.3f}) heading {:.3f}{}",
				pos.x * 0.0142875f, pos.y * 0.0142875f, pos.z * 0.0142875f, a_actor->data.angle.z, detail) : ""s);
	}

	// Recovery: pins every ragdoll body to its pose at hand-back (track 18, explicit poses: driveToPose hard-keyframes each
	// listed bone to that world transform), so while track 13's weight fades the visible pose blends from that frozen pose
	// into the animation with no physics fighting the ground. False: no usable track 18 (the caller's stiffness drive runs).
	// Track 18 is indexed by the driver's ragdoll bones, which need not be the ragdoll map's (ScanBodies) order: on a
	// super mutant a body pinned by the map's index was 1.7 m off. Asks the ragdoll interface (driver+0x70, vfunc +0x38:
	// bone -> body object, as driveToPose does) for each bone's body and re-indexes the poses by it. The object is a
	// pointer (in game), not the id: the id's offset in it is found as the one where the known body ids turn up for most
	// bones (at least half: a stray small field matching once is not it). Once per drive.
	// driveToPose applies record r to the body of remap(r): a global bone remapper (OG 0x145B04738, AE 0x143348658),
	// vfunc +0x18(remapper, context, r). Identity on humans; on a super mutant it permutes the bones (each body got
	// another bone's pose). So the poses are indexed by the r whose remapped bone's body is theirs.
	static void MapPoseBones(std::uintptr_t a_driver, void* a_context, std::int32_t a_bones, DriveState& a_state)
	{
		a_state.posesMapped = true;
		std::uintptr_t ragdoll = 0;
		if (!ReadField(a_driver, 0x70, ragdoll) || !ragdoll || a_bones <= 0) return;
		using bodyOf_t = std::uintptr_t (*)(std::uintptr_t, std::int32_t);
		const auto bodyOf = reinterpret_cast<bodyOf_t>((*reinterpret_cast<std::uintptr_t**>(ragdoll))[0x38 / 8]);
		static REL::Relocation<std::uintptr_t*> remapperPtr{ REL::ID{ 1524363, 0, 2702828 } };
		const auto remapper = *remapperPtr;
		using remap_t = std::int16_t (*)(std::uintptr_t, void*, std::uint32_t);
		const auto remap = [&](std::int32_t a_r) {
			return remapper ? reinterpret_cast<remap_t>((*reinterpret_cast<std::uintptr_t**>(remapper))[0x18 / 8])(remapper, a_context, a_r & 0xFFFF) :
			                  static_cast<std::int16_t>(a_r);
		};
		constexpr int kWords = 0x80 / 4;
		std::vector<std::array<std::uint32_t, kWords>> objects(a_bones);  // [r]: the body object keyframed by record r
		int remapped = 0;
		for (std::int32_t r = 0; r < a_bones; ++r) {
			objects[r].fill(0);
			const auto bone = remap(r);
			remapped += bone != r;
			if (bone < 0 || bone >= a_bones) continue;
			if (const auto object = bodyOf(ragdoll, bone)) SafeRead(object, objects[r].data(), sizeof(objects[r]));
		}
		const auto owner = [&](std::uint32_t a_id) {
			return std::find_if(a_state.poses.begin(), a_state.poses.end(), [&](const BonePose& a_p) { return a_p.body == a_id; });
		};
		int best = -1, bestHits = 0;
		for (int word = 0; word < kWords; ++word) {
			int hits = 0;
			for (const auto& object : objects) hits += owner(object[word]) != a_state.poses.end();
			if (hits > bestHits) best = word, bestHits = hits;
		}
		int moved = 0;
		if (bestHits * 2 >= a_bones) {
			for (std::int32_t bone = 0; bone < a_bones; ++bone)
				if (const auto pose = owner(objects[bone][best]); pose != a_state.poses.end()) {
					moved += pose->bone != bone;
					pose->bone = bone;
				}
		}
		REX::INFO("DynamicRagdollSystem: {:08X} recovery: body id at +{:#x} of the driver's body object for {}/{} bones{}; remapper {} ({} bones remapped); {} poses re-indexed"sv,
			a_state.formID, best * 4, bestHits, a_bones, bestHits * 2 >= a_bones ? "" : " (too few: map order kept)", remapper ? "on" : "off", remapped, moved);
	}

	static bool InjectExplicitPoses(std::uintptr_t a_driver, void* a_context, std::uintptr_t a_outputHolder, DriveState& a_state)
	{
		struct Header { std::int16_t capacity, numData, dataOffset, elementSize; float onFraction; std::uint8_t flags, type, pad[2]; };
		std::uintptr_t output = 0;
		std::int32_t   trackCount = 0;
		if (!ReadField(a_outputHolder, 0, output) || !output || !ReadField(output, 4, trackCount)) return false;
		auto* track = trackCount > 18 ? reinterpret_cast<Header*>(output + 0x10 + 18 * 0x10) : nullptr;
		const auto count = static_cast<std::int16_t>(a_state.poses.size());
		const bool usable = track && track->elementSize == 0x30 && track->dataOffset > 0 && track->capacity >= count && !(track->flags & 0x10);
		if (a_state.posesUsable != static_cast<std::int8_t>(usable)) {  // logged on every change: an attack's graph output may drop it
			a_state.posesUsable = static_cast<std::int8_t>(usable);
			REX::INFO("DynamicRagdollSystem: {:08X} recovery at {:.2f}s: {} bodies {} (tracks {}, explicit pose track cap {} elem {:#x} flags {:#x})"sv,
				a_state.formID, NowSeconds() - a_state.start, count, usable ? "pinned to the hand-back pose" : "NOT pinned, stiffness blend instead",
				trackCount, track ? track->capacity : 0, track ? track->elementSize : 0, track ? track->flags : 0);
		}
		if (!usable) return false;
		if (!a_state.posesMapped) MapPoseBones(a_driver, a_context, track->capacity, a_state);
		const auto* actor = RE::TESForm::GetFormByID<RE::Actor>(a_state.formID);
		auto*      data = reinterpret_cast<std::uint8_t*>(output + track->dataOffset);
		auto*      map = data + ((static_cast<std::size_t>(track->capacity) * 0x30 + 0xF) & ~std::size_t{ 0xF });  // palette index -> bone
		for (std::int16_t i = 0; i < count; ++i) {
			const auto pose = PlaceRecoveryPose(a_state.poses[i], a_state.poseAnchor, a_state.poseYaw, actor);
			std::memcpy(data + static_cast<std::size_t>(i) * 0x30, pose.translation, 0x30);
			map[i] = static_cast<std::uint8_t>(pose.bone);
		}
		track->numData = count;
		track->onFraction = 1.0f;
		return true;
	}

	void Hook_DriveToPose(void* a_driver, float a_dt, void* a_context, void* a_output)
	{
		float poseScale = 1.0f;
		bool  pinned = false;  // every body pinned to the recovery pose (track 18) this call
		{
			std::scoped_lock lock{ g_driveMutex };
			if (const auto it = g_drives.find(reinterpret_cast<std::uintptr_t>(a_driver)); it != g_drives.end()) {
				auto& state = it->second;
				if (state.calls++ == 0) UnlockRagdollRoot(reinterpret_cast<std::uintptr_t>(a_driver), state.formID);
				poseScale = state.poseScale;
				const float now = NowSeconds();
				const float recover = std::clamp((now - state.start) / kRecoverTime, 0.0f, 1.0f);
				const float smooth = recover * recover * (3.0f - 2.0f * recover);
				// During a melee attack (e.g. a full-body machete swing) a very soft ragdoll lags the fast animation
				// and gets wrenched into odd poses; keep it firmer and let it catch up faster.
				const auto* actor = RE::TESForm::GetFormByID<RE::Actor>(state.formID);
				const bool attacking = actor && actor->meleeAttackState != 0;
				const float hitSoftness = actor && actor->IsPlayerRef()                     ? kPlayerHitSoftness :
				                          actor && RE::PowerArmor::ActorInPowerArmor(*actor) ? kPowerArmorHitSoftness :
				                                                                               kHitSoftness;
				const float softness = attacking ? std::max(hitSoftness, kAttackSoftness) : hitSoftness;
				const float stiffness = state.burning ? kBurnStiffness * (1.0f - 0.6f * state.burnProgress) :
				                        state.recover ? kRecoverStiffness :
				                                        softness + (1.0f - softness) * smooth;
				const float catchUp = attacking ? std::max(kMaxCatchUpSpeed, kAttackCatchUpSpeed) : kMaxCatchUpSpeed;
				if (attacking && !state.attackLogged) {
					state.attackLogged = true;
					REX::INFO("DynamicRagdollSystem: {:08X} reaction during a melee attack (state {}): firmer, faster catch-up"sv,
						state.formID, static_cast<std::uint32_t>(actor->meleeAttackState));
				}
				// Recovery: the animation takes over across the whole blend (eased), not in the last fBlendOut (a snap).
				const float fadeOut = std::clamp((state.end - now) / std::max(state.end - state.start, 0.01f), 0.0f, 1.0f);
				const float weight = state.recover ? fadeOut * fadeOut * (3.0f - 2.0f * fadeOut) : std::clamp((state.end - now) / kBlendOut, 0.0f, 1.0f);
				if (a_dt > 0.0f && InjectRigidBodyControls(reinterpret_cast<std::uintptr_t>(a_output), stiffness, weight, a_dt, catchUp,
						state.burning ? kBurnArmStiffness : -1.0f, state)) {
					++state.injected;
					if (state.recover && !state.poses.empty()) pinned = InjectExplicitPoses(reinterpret_cast<std::uintptr_t>(a_driver), a_context, reinterpret_cast<std::uintptr_t>(a_output), state);
				}
			}
		}
		// driveToPose builds its targets from the local pose (track 2, hkQsTransform 0x30 bytes per bone) without the
		// skeleton root's scale (race height): a 1.72x giant was pulled down to human size, a 0.72x NPC up to it. Scale
		// every bone offset for the call (uniform scale: the whole model-space pose scales about the root), then restore.
		thread_local std::vector<float> saved;
		float*       pose = nullptr;
		std::int32_t count = 0;
		std::uintptr_t output = 0;
		std::int32_t   trackCount = 0;
		if (std::abs(poseScale - 1.0f) > 0.01f && ReadField(reinterpret_cast<std::uintptr_t>(a_output), 0, output) && output &&
			ReadField(output, 4, trackCount) && trackCount > 2 && !(*reinterpret_cast<std::uint8_t*>(output + 0x3C) & 0x10)) {
			count = std::clamp<std::int32_t>(*reinterpret_cast<std::int16_t*>(output + 0x32), 0, 256);
			pose = reinterpret_cast<float*>(output + *reinterpret_cast<std::int16_t*>(output + 0x34));
			saved.resize(static_cast<std::size_t>(count) * 3);
			for (std::int32_t i = 0; i < count; ++i)
				for (std::int32_t k = 0; k < 3; ++k) {
					saved[i * 3 + k] = pose[i * 12 + k];
					pose[i * 12 + k] *= poseScale;
				}
		}
		// The ragdoll skeleton's child bones are lockTranslation (RE_NOTES_AE.md): the mapping gives them the reference-pose
		// translation, human length whatever the skeleton scale. Scaled track 2 alone moved the pelvis but kept human-length
		// limbs (1.72x: crouched in midair; small: arms spread). Scale the reference pose for the call too. The skeleton is
		// shared by every actor of the race: scaled calls run alone, every other call holds the shared lock.
		static std::shared_mutex refPoseMutex;
		std::shared_lock         sharedLock{ refPoseMutex, std::defer_lock };
		std::unique_lock         uniqueLock{ refPoseMutex, std::defer_lock };
		float*                   refPose = nullptr;
		std::int32_t             refCount = 0;
		if (pose) {
			const std::uintptr_t skeleton = RagdollSkeleton(reinterpret_cast<std::uintptr_t>(a_driver));
			std::uintptr_t       refData = 0;
			std::int32_t         boneCount = 0;
			// hkaSkeleton m_referencePose: hkArray<hkQsTransform> at +0x38 (data, count +0x40); count must match the bones (+0x30).
			if (skeleton && ReadField(skeleton, 0x38, refData) && refData && ReadField(skeleton, 0x40, refCount) &&
				ReadField(skeleton, 0x30, boneCount) && refCount == boneCount && refCount > 0 && refCount <= kMaxRagdollBones) {
				refPose = reinterpret_cast<float*>(refData);
			} else {
				static bool logged = false;
				if (!logged) {
					logged = true;
					REX::INFO("DynamicRagdollSystem: ragdoll reference pose unreadable (skeleton {:X}, ref count {} bones {}); limb lengths not scaled"sv,
						skeleton, refCount, boneCount);
				}
			}
		}
		thread_local std::vector<float> savedRef;
		if (refPose) {
			uniqueLock.lock();
			savedRef.resize(static_cast<std::size_t>(refCount) * 3);
			for (std::int32_t i = 0; i < refCount; ++i)
				for (std::int32_t k = 0; k < 3; ++k) {
					savedRef[i * 3 + k] = refPose[i * 12 + k];
					refPose[i * 12 + k] *= poseScale;
				}
		} else {
			sharedLock.lock();
		}
		g_driveToPose(a_driver, a_dt, a_context, a_output);
		for (std::int32_t i = 0; refPose && i < refCount; ++i)
			for (std::int32_t k = 0; k < 3; ++k) refPose[i * 12 + k] = savedRef[i * 3 + k];
		// Every bone keyframed: driveToPose sets driver+0xBB and postPhysics (0x1416FF3A0) then skips the ragdoll blend,
		// so the animation showed at once. Cleared here, between the two, the pinned pose still blends into the animation.
		// (A hand left unpinned instead hung at odd wrist angles and twisted the forearm/finger bones.)
		if (pinned) *(reinterpret_cast<std::uint8_t*>(a_driver) + 0xBB) = 0;
		for (std::int32_t i = 0; pose && i < count; ++i)
			for (std::int32_t k = 0; k < 3; ++k) pose[i * 12 + k] = saved[i * 3 + k];
	}


	void Hook_MovementRagdollQuery(void* a_manager, char** a_result)
	{
		g_ragdollActiveQuery(a_manager, a_result);
		if (!a_result || !*a_result) return;
		std::scoped_lock lock{ g_driveMutex };
		for (auto& [driver, state] : g_drives) {
			if (state.manager == reinterpret_cast<std::uintptr_t>(a_manager)) {
				++state.movementQueries;
				if (**a_result) {
					++state.movementCleared;
					**a_result = 0;
				}
				return;
			}
		}
	}


	bool Hook_IsRagdollActive(void* a_graph)
	{
		const auto ret = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
		if (ret == g_graphUpdateReturn || ret == g_movementReturn) {
			std::uintptr_t driver = 0;
			if (ReadField(reinterpret_cast<std::uintptr_t>(a_graph), 0x208, driver) && driver) {
				std::scoped_lock lock{ g_driveMutex };
				if (const auto drive = g_drives.find(driver); drive != g_drives.end()) {
					if (ret == g_movementReturn) {  // AE movement query (OG: Hook_MovementRagdollQuery)
						++drive->second.movementQueries;
						if ((*g_ragdollActiveHook)(a_graph)) ++drive->second.movementCleared;
					}
					return false;
				}
			}
		}
		return (*g_ragdollActiveHook)(a_graph);
	}


	// `call [rax+0x98]` (graph vtable slot 19) at a_call; returns the address after it, or 0 on mismatch.
	std::uintptr_t CheckSlot19Call(std::uintptr_t a_call)
	{
		static constexpr std::uint8_t callSlot19[6] = { 0xFF, 0x90, 0x98, 0x00, 0x00, 0x00 };
		std::uint8_t bytes[6]{};
		if (!SafeRead(a_call, bytes, sizeof(bytes)) || std::memcmp(bytes, callSlot19, sizeof(bytes)) != 0) return 0;
		return a_call + sizeof(bytes);
	}


	// Hooks graph vtable slot 19 once; callers set their return address first.
	bool EnableRagdollActiveHook()
	{
		if (g_ragdollActiveHook) return true;
		std::uintptr_t slot19 = 0;
		if (!ReadField(RE::VTABLE::BShkbAnimationGraph[0].address(), 19 * 8, slot19) ||
			slot19 != REL::ID{ 609532, 0, 2256904 }.address()) return false;
		static REL::THookVFT<bool(void*)> hook{ REL::Offset{ RE::VTABLE::BShkbAnimationGraph[0].offset() }, 19, &Hook_IsRagdollActive };
		hook.Init();
		hook.Enable();
		g_ragdollActiveHook = &hook;
		return true;
	}


	void InstallGraphUpdateHook()
	{
		// Graph per-frame update (ID 1273937, OG 0x14161D9A0): its "ragdoll active?" call.
		const auto call = REL::ID{ 1273937, 0, 2256850 }.address() + REL::Offset{ 0x54, 0, 0x69 }.offset();
		const auto ret = CheckSlot19Call(call);
		if (!ret || !EnableRagdollActiveHook()) {
			REX::INFO("DynamicRagdollSystem: graph update call site mismatch; NPCs may keep their hit-time velocity"sv);
			return;
		}
		g_graphUpdateReturn = ret;
	}


	void InstallMovementHook()
	{
		if (REX::FModule::IsRuntimeAE()) {
			// The visitor is inlined into the movement update: its slot-19 call is at + 0x4ED.
			const auto ret = CheckSlot19Call(REL::ID{ 737625, 0, 2229934 }.address() + 0x4ED);
			if (!ret || !EnableRagdollActiveHook()) {
				REX::INFO("DynamicRagdollSystem: movement call site mismatch; NPCs will stop during reactions"sv);
				return;
			}
			g_movementReturn = ret;
			return;
		}
		const auto site = REL::ID{ 737625 }.address() + 0x3B8;
		const auto target = REL::ID{ 453608 }.address();
		std::uint8_t bytes[5]{};
		std::int32_t rel = 0;
		if (!SafeRead(site, bytes, 5) || bytes[0] != 0xE8 || (std::memcpy(&rel, bytes + 1, 4), site + 5 + rel != target)) {
			REX::INFO("DynamicRagdollSystem: movement call site mismatch; NPCs will stop during reactions"sv);
			return;
		}
		g_ragdollActiveQuery = reinterpret_cast<RagdollActiveQuery>(REL::GetTrampoline().write_call<5>(site, &Hook_MovementRagdollQuery));
	}


	void InstallDriveToPoseHook()
	{
		// Call site in the behavior graph pre-physics update: ID 1348502 + 0x12F, `call driveToPose` (ID 993992).
		const auto site = REL::ID{ 1348502, 0, 2256836 }.address() + REL::Offset{ 0x12F, 0, 0x144 }.offset();
		const auto target = REL::ID{ 993992, 0, 2258034 }.address();
		std::uint8_t bytes[5]{};
		std::int32_t rel = 0;
		if (!SafeRead(site, bytes, 5) || bytes[0] != 0xE8 || (std::memcpy(&rel, bytes + 1, 4), site + 5 + rel != target)) {
			REX::INFO("DynamicRagdollSystem: driveToPose call site mismatch; plugin inactive"sv);
			return;
		}
		g_driveToPose = reinterpret_cast<DriveToPose>(REL::GetTrampoline().write_call<5>(site, &Hook_DriveToPose));
	}


	// postPhysics copies the ragdoll pose onto the skeleton. It keeps each bone's scale but takes bone offsets from
	// the ragdoll bodies, whose joints are adult-sized, so a child (or any shrunk/enlarged human skeleton) took on
	// adult proportions in any ragdoll (reaction, knockdown, death). When the offsets disagree, put the animation's
	// back (rotations stay).
	void Hook_PostPhysics(void* a_driver, void* a_context, std::uint32_t a_arg, void* a_output)
	{
		const auto driver = reinterpret_cast<std::uintptr_t>(a_driver);
		// Track 2 = local pose, hkQsTransform {translation, rotation, scale} = 0x30 bytes per bone.
		std::uintptr_t output = 0;
		std::int32_t   trackCount = 0;
		float*         pose = nullptr;
		std::int32_t   count = 0;
		if (ReadField(reinterpret_cast<std::uintptr_t>(a_output), 0, output) && output && ReadField(output, 4, trackCount) &&
			trackCount > 2 && !(*reinterpret_cast<std::uint8_t*>(output + 0x3C) & 0x10)) {
			count = std::clamp<std::int32_t>(*reinterpret_cast<std::int16_t*>(output + 0x32), 0, 256);
			pose = reinterpret_cast<float*>(output + *reinterpret_cast<std::int16_t*>(output + 0x34));
		}
		thread_local std::vector<Vec3> before;
		before.resize(count);
		for (std::int32_t i = 0; i < count; ++i) before[i] = { pose[i * 12], pose[i * 12 + 1], pose[i * 12 + 2] };

		g_postPhysics(a_driver, a_context, a_arg, a_output);
		if (!count) return;

		// Compare only the bones postPhysics rewrote (the ragdoll's ~18): summing the whole skeleton let the
		// untouched fingers/face dilute a child's stretch under the threshold. Bones are parent-first, so the
		// first rewritten one is the ragdoll root; its offset places the body (death/knockdown), so it stays.
		std::int32_t first = -1;
		float animLength = 0.0f, ragdollLength = 0.0f;
		for (std::int32_t i = 0; i < count; ++i) {
			const Vec3 after{ pose[i * 12], pose[i * 12 + 1], pose[i * 12 + 2] };
			const auto d = Sub(after, before[i]);
			if (Dot(d, d) < 1.0e-8f) continue;
			if (first < 0) {
				first = i;
				continue;
			}
			animLength += Length(before[i]);
			ragdollLength += Length(after);
		}
		// Adult skeletons stay within a few percent; ragdoll stretch alone should not trip this. Once a driver trips
		// it stays latched: a child hovers near the threshold, and frames just under it showed the stretch.
		// shortcut: latched per driver address, never cleared; a new actor reusing the address keeps animation bone
		// lengths too (same bone lengths on an adult human, so harmless there).
		static std::unordered_set<std::uintptr_t> latched;
		{
			const bool tripped = animLength > 0.0f && std::abs(ragdollLength / animLength - 1.0f) >= 0.1f;
			std::scoped_lock lock{ g_driveMutex };
			if (!latched.contains(driver)) {
				if (!tripped) return;
				latched.insert(driver);
				REX::INFO("DynamicRagdollSystem: driver {:X} skeleton sized unlike its ragdoll (bone offsets x{:.2f}); keeping animation bone lengths"sv,
					driver, ragdollLength / animLength);
			}
		}
		for (std::int32_t i = first + 1; i < count; ++i) {
			pose[i * 12] = before[i].x;
			pose[i * 12 + 1] = before[i].y;
			pose[i * 12 + 2] = before[i].z;
		}
	}


	void InstallPostPhysicsHook()
	{
		// Graph post-physics update (ID 1414229) + 0x1EA: `call hkbRagdollDriver::postPhysics` (ID 560275).
		const auto site = REL::ID{ 1414229, 0, 2256837 }.address() + REL::Offset{ 0x1EA, 0, 0x1DC }.offset();
		const auto target = REL::ID{ 560275, 0, 2258035 }.address();
		std::uint8_t bytes[5]{};
		std::int32_t rel = 0;
		if (!SafeRead(site, bytes, 5) || bytes[0] != 0xE8 || (std::memcpy(&rel, bytes + 1, 4), site + 5 + rel != target)) {
			REX::INFO("DynamicRagdollSystem: postPhysics call site mismatch; small skeletons stretch to adult size when ragdolled"sv);
			return;
		}
		g_postPhysics = reinterpret_cast<PostPhysics>(REL::GetTrampoline().write_call<5>(site, &Hook_PostPhysics));
	}


}
