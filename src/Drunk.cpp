#include "Walker.h"

// Drunk mode ([Player] bDrunk, key iDrunkKey): the player in the engine knockdown ragdoll, held down with Paralysis
// (verified in game: the knockdown, camera follow and the Pip-Boy/activate/movement block all work on the player).
// The death walk's walker keeps it upright and walks it with WASD relative to the camera; after a fall it gets itself up
// (port of RagdollLab src/Puppet.cpp GetUp); on release it recovers into animation (RagdollLab Recover).

namespace DynamicRagdollSystem
{
	namespace
	{
		using namespace walker;

		constexpr float kHalfPi = 0.5f * kPi;
		constexpr float kGameToPhysics = 0.0142875f, kPhysicsToGame = 69.99125f;
		// RagdollLab get-up timings and poses (Puppet.cpp, constants there too).
		constexpr float kRollTime = 1.2f, kKneelTime = 1.2f, kLungeTime = 0.9f, kStandTime = 1.0f, kLungeReach = 0.55f;
		constexpr float kEaseIn = 0.5f, kRollLimbs = 0.2f, kArmKp = 4.0f;
		constexpr float kLungeStart = 0.8f, kLean = 0.25f, kArmReach = 0.65f, kArmSpeed = 2.0f;
		constexpr float kSetupWait = 2.0f;  // s after the key to find a usable skeleton (in game: queued → ragdoll took 0.57 s)

		struct Puppet
		{
			RE::bhkWorld* world{};
			Stumble       scan;            // ragdoll bodies (ScanBodies once they are dynamic)
			Walker        w;
			float         started{};
			bool          paralyzed{};
			bool          ready{};         // walker built from the first dynamic pose
			bool          release{};       // let go once back on its feet
			bool          knocked{};       // knockdown started (a frame later when the camera had to leave first person)
			float         sway{ 1.0f }, swayFrom{ 1.0f }, swayTo{ 1.0f }, swayStart{}, swayLength{};  // walking support factor
			const char*   stop{};          // set in the Havok step, acted on next frame (Paralysis is not changed mid-step)
		};
		std::optional<Puppet> g_drunk;  // main thread only (frame update and Havok step)

		RE::Actor* Player() { return RE::PlayerCharacter::GetSingleton(); }

		bool KeyDown(int a_vk) { return (GetAsyncKeyState(a_vk) & 0x8000) != 0; }

		// Game window in front, not paused, no console or Pip-Boy.
		bool InputAllowed()
		{
			DWORD pid = 0;
			GetWindowThreadProcessId(GetForegroundWindow(), &pid);
			const auto* ui = RE::UI::GetSingleton();
			return pid == GetCurrentProcessId() && ui && !ui->GameIsPaused() && !ui->GetMenuOpen("Console") && !ui->GetMenuOpen("PipboyMenu");
		}

		// Edge-triggered: true once per press (RagdollLab main.cpp Pressed).
		bool Pressed(int a_vk)
		{
			static bool down = false;
			const bool  now = KeyDown(a_vk);
			const bool  pressed = now && !down;
			down = now;
			return pressed && InputAllowed();
		}

		void SetParalysis(RE::Actor* a_actor, bool a_on)
		{
			auto* values = RE::ActorValue::GetSingleton();
			if (!values || !values->paralysis || !a_actor) return;
			// Paralysis > 0 makes the knockdown skip its get-up (Actor+0x2D0 bit 31). Temporary modifiers are saved: always
			// given back (StopDrunk runs before every save and load).
			a_actor->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *values->paralysis, a_on ? 1.0f : -1.0f);
		}

		const char* RiseName(Rise a_rise)
		{
			switch (a_rise) {
			case Rise::kDown: return "down";
			case Rise::kRoll: return "rolling over";
			case Rise::kKneel: return "kneeling up";
			case Rise::kLunge: return "foot forward";
			case Rise::kStand: return "standing up";
			default: return "walking";
			}
		}

		// Starts and ends at rest like Smooth, but most of the way is covered early: an effort, then a settle.
		float Effort(float a_t) { return Smooth(std::pow(std::clamp(a_t, 0.0f, 1.0f), 0.85f)); }

		// Facing a_yaw, tipped forward by a_angle about its own right axis (pi/2 = lying on the belly, head ahead).
		Mat3 Pitched(float a_yaw, float a_angle) { return Multiply(AxisAngle(Rotate(Yaw(a_yaw), kX), -a_angle), Yaw(a_yaw)); }

		// Getting up after a fall (RagdollLab GetUp, weak-mode branches dropped). FO4's hips bend only ~30-45 degrees and
		// the knee one way, so: lie still -> roll onto the belly -> pivot up about the knees to kneeling -> swing one foot
		// forward -> rise over it while the back foot steps in -> hand back to the walker. True while down or getting up.
		bool GetUp(Ctx& c)
		{
			auto&       s = c.s;
			const auto  pelvis = c.Pos(s.trunk[0].id);
			const auto  frame = c.BodyFrame();
			const auto  belly = Rotate(frame, kY);
			const float height = pelvis.z - s.ground;
			const auto  begin = [&](Rise a_stage) {
				if (a_stage != s.rise) REX::INFO("DynamicRagdollSystem: 00000014 drunk: {} (pelvis {:.2f} up)"sv, RiseName(a_stage), height);
				s.rise = a_stage;
				s.stageStart = s.time;
				s.stageFrom = pelvis;
			};

			if (s.rise == Rise::kNone) {
				if (height > 0.55f * s.standHeight && Rotate(frame, kZ).z > 0.5f) return false;
				s.swing = -1;
				begin(Rise::kDown);  // the walker lets go: holding the trunk up by grounded feet would hoist a lying body
			}
			const float stage = s.time - s.stageStart;
			// Stages whose targets start away from the body ease in; the foot swings start where the foot is.
			c.strength = s.rise == Rise::kRoll || s.rise == Rise::kKneel ? Smooth(stage / kEaseIn) : 1.0f;

			// Supports: every limb body touching the ground. The legs and arms being moved are taken out below.
			std::vector<std::uint32_t> ground;
			const auto isTrunk = [&](std::uint32_t a_id) {
				return std::any_of(s.trunk.begin(), s.trunk.end(), [&](const Part& t) { return t.id == a_id; });
			};
			for (const auto& [id, shape] : s.shapes)
				if (!isTrunk(id) && c.Body(id) && OnGround(c, id)) ground.push_back(id);
			const auto without = [&](int a_foot) {
				const auto& f = s.feet[a_foot];
				std::erase_if(ground, [&](std::uint32_t p) { return p == f.id || p == f.calf || p == f.thigh; });
			};
			const auto support = [&](Vec3 a_reaction) {
				if (!ground.empty()) Brace(c, a_reaction, ground);
			};
			const auto  fwd = Rotate(Yaw(s.riseYaw), kY), right = Rotate(Yaw(s.riseYaw), kX);
			const float kneeRadius = s.shapes.contains(s.feet[0].calf) ? s.shapes[s.feet[0].calf].radius : 0.07f;
			const float thigh = 0.5f * (s.feet[0].thighLength + s.feet[1].thighLength);
			const auto  startKneel = [&] {
				s.pivot = Scale(Add(Flat(c.Pos(s.feet[0].calf)), Flat(c.Pos(s.feet[1].calf))), 0.5f);  // knees: the body pivots up about them
				const auto lie = Flat(Sub(pelvis, s.pivot));
				s.riseYaw = YawOf(Length(lie) > 0.1f ? lie : Flat(Sub(c.Pos(s.head), pelvis)));
				// Hands go down beside the chest: where each shoulder lies once face down (also from a roll that stopped on the side).
				const auto back = Scale(Rotate(Yaw(s.riseYaw), kY), 0.1f);
				for (const auto& a : s.limbs)
					if (a.id == a.shoulder)
						s.hands[a.side] = Add(Sub(Flat(Add(pelvis, Rotate(Pitched(s.riseYaw, kHalfPi), Rotate(Transpose(frame), Sub(c.Pos(a.id), pelvis))))), back),
							Vec3{ 0.0f, 0.0f, s.ground });
				begin(Rise::kKneel);
			};
			// Kneeling up: the knee-to-hip angle above the floor a_t into a rise lasting a_time, and the hips at that angle.
			const auto kneelAngle = [&](float a_t, float a_time) { return Smooth(a_t / a_time) * kHalfPi; };
			const auto kneelHips = [&](float a_theta) {
				auto hips = Add(s.pivot, Scale(Add(Scale(fwd, std::cos(a_theta)), Scale(kZ, std::sin(a_theta))), thigh));
				hips.z += s.ground + kneeRadius;
				return hips;
			};
			// A foot swung along a lifted arc (as in walking), its push taken from the pelvis.
			const auto swingFoot = [&](int a_foot, Vec3 a_to, float a_phase) {
				auto arc = Add(s.swingFrom, Scale(Sub(a_to, s.swingFrom), Smooth(a_phase)));
				arc.z += kWalkStepLift * std::sin(kPi * std::min(1.0f, a_phase * 1.25f));
				c.Push(s.trunk[0].id, Scale(DriveLeg(c, a_foot, arc, fwd), -1.0f));
				without(a_foot);
			};
			// Arms straightened from the shoulders along a_dir(shoulder, side) on a slow spring (kArmKp). Returns the impulse
			// given; arms moved this way stop counting as supports unless a_planted (hands pushing on the floor).
			const auto moveArms = [&](auto a_dir, bool a_planted) {
				Vec3 impulse{};
				for (const auto& a : s.limbs) {
					MotionState m{}, sm{};
					if (!a.shoulder || !c.Read(a.id, m)) continue;
					const auto from = c.Pos(a.shoulder);
					// Carried along with the shoulder (left behind, they drag the rising chest down), except in the roll.
					Vec3 carried{};
					if (s.rise != Rise::kRoll && c.Read(a.shoulder, sm))
						carried = Add(sm.linear, Cross(Rotate(sm.orientation, sm.angular), Sub(from, c.Com(a.shoulder))));
					const auto drift = Clamp(Scale(Sub(Add(from, Scale(a_dir(from, a.side), a.reach)), c.Com(a.id)), kArmKp), kArmSpeed);
					auto       dv = Clamp(Sub(Add(drift, carried), m.linear), kWalkSwingAccel * c.strength * c.dt);
					dv.z += c.gravity * c.dt;
					c.AddVelocity(a.id, m, dv);
					impulse = Add(impulse, Scale(dv, 1.0f / m.inverseMass));
				}
				std::erase_if(ground, [&](std::uint32_t p) {
					return std::any_of(s.limbs.begin(), s.limbs.end(), [&](const Arm& a) { return a.id == p && !(a_planted && a.hand); });
				});
				return impulse;
			};
			// Hands down by the thighs, clear of them: the pose they hang in anyway.
			const auto atSides = [&](Vec3 a_shoulder, int) {
				const auto out = Dot(Sub(a_shoulder, pelvis), right) > 0.0f ? right : Scale(right, -1.0f);
				return UnitOr(Sub(Scale(out, 0.1f), kZ), Scale(kZ, -1.0f));
			};
			const auto toward = [&](Vec3 a_from, Vec3 a_to) { return UnitOr(Sub(a_to, a_from), Scale(kZ, -1.0f)); };

			switch (s.rise) {
			case Rise::kDown: {  // limp until the body lies still
				if (stage < 0.5f || (std::max(Length(c.Vel(s.trunk[0].id)), Length(c.Vel(s.head))) > 0.5f && stage < 2.0f)) return true;
				s.riseYaw = YawOf(Flat(Sub(c.Pos(s.head), pelvis)));
				if (belly.z < -0.2f) startKneel();  // already more face down than up
				else {
					// How far the belly has to turn about the body's length to face down, the way it already leans.
					const float now = std::atan2(Dot(belly, Rotate(Yaw(s.riseYaw), kX)), belly.z);
					s.rollTurn = (now >= 0.0f ? kPi : -kPi) - now;
					begin(Rise::kRoll);
				}
				return true;
			}

			case Rise::kRoll: {  // the target turns steadily to face down, so the trunk is never pulled far
				const float rollTime = kRollTime * std::abs(s.rollTurn) / kPi;
				const float t = Smooth(stage / rollTime);
				const auto  target = Multiply(AxisAngle(fwd, s.rollTurn * (t - 1.0f)), Pitched(s.riseYaw, kHalfPi));
				auto        lying = Flat(pelvis);
				lying.z = s.ground + (s.shapes.contains(s.trunk[0].id) ? s.shapes[s.trunk[0].id].radius : 0.16f);
				if (ground.empty()) return true;
				auto reaction = HoldTrunk(c, lying, target, {});
				c.strength *= kRollLimbs;
				reaction = Add(reaction, moveArms([&](Vec3, int) { return fwd; }, false));  // overhead, as in a log roll
				// Until on its side: the far leg swings across the near one and turns the hips along.
				if (belly.z > 0.3f) {
					const auto across = Flat(Rotate(Multiply(AxisAngle(fwd, s.rollTurn > 0.0f ? -kHalfPi : kHalfPi), Pitched(s.riseYaw, kHalfPi)), kY));
					const int  farLeg = Dot(c.Pos(s.feet[0].thigh), across) < Dot(c.Pos(s.feet[1].thigh), across) ? 0 : 1;
					c.Push(s.trunk[0].id,
						Scale(DriveLeg(c, farLeg, Add(Add(c.Pos(s.feet[1 - farLeg].id), Scale(across, 0.2f)), Vec3{ 0.0f, 0.0f, 0.15f }), across), -1.0f));
					without(farLeg);
				}
				support(reaction);
				if ((t >= 1.0f && belly.z < -0.5f) || stage > rollTime + 1.0f) startKneel();
				return true;
			}

			case Rise::kKneel: {  // hands push the chest up as thighs and trunk swing up about the knees; the shins stay down
				const float theta = kneelAngle(stage, kKneelTime);
				const auto  target = Pitched(s.riseYaw, kHalfPi - theta);
				const auto  hips = kneelHips(theta);
				Vec3        reaction{};
				if (std::abs(Rotate(frame, kX).z) < 0.5f)  // not while still on its side: arms driven there pin it down
					reaction = moveArms([&](Vec3 a_shoulder, int a_side) {
						const auto toHand = Sub(s.hands[a_side], a_shoulder);
						return Length(toHand) < kArmReach ? Scale(toHand, 1.0f / kArmReach) : atSides(a_shoulder, a_side);
					}, true);
				if (!ground.empty()) {
					reaction = Add(reaction, HoldTrunk(c, hips, target, {}));
					for (const auto& f : s.feet) {  // each thigh along its knee-to-hip line
						MotionState m{};
						if (!c.Read(f.thigh, m)) continue;
						const auto knee = c.Pos(f.calf), hip = Add(hips, Rotate(target, { f.offset.x, 0.0f, 0.0f }));
						auto dv = Clamp(Sub(Scale(Sub(Add(hip, Scale(Sub(knee, hip), f.thighCom)), c.Com(f.thigh)), kWalkSwingKp), m.linear),
							kWalkSwingAccel * c.strength * c.dt);
						dv.z += c.gravity * c.dt;
						c.AddVelocity(f.thigh, m, dv);
						reaction = Add(reaction, Scale(dv, 1.0f / m.inverseMass));
					}
				}
				support(reaction);
				if ((stage > kLungeStart * kKneelTime && Rotate(frame, kZ).z > 0.9f) || stage > kKneelTime + 0.3f) {
					if (Rotate(frame, kZ).z < 0.8f) begin(Rise::kDown);  // didn't make it: lie still and retry
					else {
						s.front = 1 - static_cast<int>(s.lunges++ % 2);  // alternate the leading leg every attempt
						s.swingFrom = c.Pos(s.feet[s.front].id);
						begin(Rise::kLunge);
					}
				}
				return true;
			}

			case Rise::kLunge: {  // the hips finish kneeling up while one foot swings forward and plants
				const auto& f = s.feet[s.front];
				const float phase = std::min(1.0f, stage / kLungeTime);
				auto        to = Add(Add(s.pivot, Scale(fwd, kLungeReach)), Scale(right, f.offset.x));
				to.z = s.ground + f.offset.z;
				swingFoot(s.front, to, phase);
				// As the foot comes down, the hand on its side goes to its knee, ready to push on it.
				const auto knee = c.Pos(f.calf);
				auto       reaction = moveArms([&](Vec3 a_shoulder, int a_side) {
					return phase > 0.5f && Dot(Sub(a_shoulder, pelvis), right) * f.offset.x > 0.0f ? toward(a_shoulder, knee) : atSides(a_shoulder, a_side);
				}, false);
				const auto hips = kneelHips(kneelAngle(stage + kLungeStart * kKneelTime, kKneelTime));  // the kneel carried on
				if (!ground.empty()) support(Add(reaction, HoldTrunk(c, hips, Yaw(s.riseYaw), {})));
				if (phase >= 0.9f && OnGround(c, f.id)) begin(Rise::kStand);
				else if (stage > 3.0f * kLungeTime || Rotate(frame, kZ).z < 0.7f) begin(Rise::kDown);  // lie still and retry
				return true;
			}

			case Rise::kStand: {  // the pelvis pushes up over the front foot, chest forward, straightening last; the back foot steps in
				const float t = Effort(stage / kStandTime);
				const auto& front = s.feet[s.front];
				const auto& back = s.feet[1 - s.front];
				auto        over = Sub(Sub(Flat(c.Pos(front.id)), Scale(right, front.offset.x)), Scale(fwd, front.offset.y));
				over.z = s.ground + s.standHeight * kWalkCrouch;
				const auto knee = c.Pos(front.calf);
				auto       reaction = moveArms([&](Vec3 a_shoulder, int a_side) {  // pushing on the front knee for the first part
					return stage < 0.4f * kStandTime && Dot(Sub(a_shoulder, pelvis), right) * front.offset.x > 0.0f ? toward(a_shoulder, knee) :
					                                                                                                     atSides(a_shoulder, a_side);
				}, false);
				if (stage / kStandTime > 0.4f) {
					const float phase = std::min(1.0f, (stage / kStandTime - 0.4f) / 0.6f);
					if (s.swingFrom.z > 1.0e8f) s.swingFrom = c.Pos(back.id);
					auto to = Add(Add(over, Scale(right, back.offset.x)), Scale(fwd, back.offset.y));
					to.z = s.ground + back.offset.z;
					swingFoot(1 - s.front, to, phase);
				} else {
					s.swingFrom = { 0.0f, 0.0f, 1.0e9f };  // set when the back foot starts
				}
				// Chest forward over the front foot while rising, straightened last.
				const auto upright = Pitched(s.riseYaw, kLean * std::sin(kPi * std::clamp((stage / kStandTime - 0.2f) / 0.75f, 0.0f, 1.0f)));
				if (!ground.empty()) support(Add(reaction, HoldTrunk(c, Add(s.stageFrom, Scale(Sub(over, s.stageFrom), t)), upright, {})));
				if (stage > kStandTime && height > 0.8f * s.standHeight) {  // up: the walker takes over from here
					s.root = pelvis;
					s.yaw = s.riseYaw;
					s.swing = s.lastSwing = -1;
					s.landed = s.time;
					s.rise = Rise::kNone;
					++s.getUps;
					s.upAt = s.time;
					REX::INFO("DynamicRagdollSystem: 00000014 drunk: back on its feet ({} get-ups)"sv, s.getUps);
					return false;
				}
				if (stage > kStandTime + 2.0f || Rotate(frame, kZ).z < 0.7f) begin(Rise::kDown);
				return true;
			}

			default: return false;
			}
		}

		// Builds the walker from the first dynamic frame (the bodies still hold the standing pose then). Havok step,
		// engine holds the world write lock. False: not yet (scan again next step).
		bool BuildWalker(Puppet& a_p, RE::Actor* a_player, RE::hknpBSWorld* a_world)
		{
			ScanBodies(a_p.scan, a_player, a_world);
			auto&                       worldBodies = a_world->m_bodyManager.m_bodies;
			std::vector<BodyTarget>     targets;
			std::vector<const BodyRef*> refs;
			Ctx                         c{ a_world, a_p.w, {}, 0.0f, 0.0f };
			for (const auto& body : a_p.scan.bodies) {
				const auto* b = FindBody(worldBodies, body);
				if (!b) continue;
				targets.push_back({ body.id.m_value, RotationFrom(b->m_transform.m_rotation), BodyPosition(*b) });
				refs.push_back(&body);
				c.bodies[body.id.m_value] = b;
			}
			const float ground = a_p.w.ground;
			a_p.w = Walker{};
			a_p.w.ground = ground;
			if (!Setup(targets, a_p.started, a_player->data.angle.z, refs, c, a_p.w)) return false;
			// Get-up data: every arm body with its shoulder (the arm body on its side hanging off the trunk), reach along the arm.
			const auto refOf = [&](std::uint32_t a_id) -> const BodyRef* {
				for (const auto* r : refs)
					if (r->id.m_value == a_id) return r;
				return nullptr;
			};
			for (const auto* r : refs) {
				if (!r->arm) continue;
				const auto* parent = refOf(r->parentId.m_value);
				const bool  upper = !parent || !parent->arm;
				Arm         arm{ r->id.m_value, 0, r->hand ? 0.6f : upper ? 0.15f : 0.4f, r->side == 'R' ? 1 : 0, r->hand };
				for (const auto* o : refs) {
					const auto* op = refOf(o->parentId.m_value);
					if (o->arm && o->side == r->side && (!op || !op->arm)) arm.shoulder = o->id.m_value;
				}
				a_p.w.limbs.push_back(arm);
			}
			a_p.w.head = std::max_element(a_p.w.trunk.begin(), a_p.w.trunk.end(), [](const Part& a, const Part& b) { return a.offset.z < b.offset.z; })->id;
			REX::INFO("DynamicRagdollSystem: 00000014 drunk: {} trunk bodies, {} arm bodies, {} capsules, pelvis {:.2f} up, legs {:.2f}/{:.2f}"sv,
				a_p.w.trunk.size(), a_p.w.limbs.size(), a_p.w.shapes.size(), a_p.w.standHeight, a_p.w.feet[0].thighLength, a_p.w.feet[0].shinLength);
			return true;
		}

		bool Recover(RE::Actor* a_actor, const Puppet& a_p)
		{
			if (!a_p.ready) return false;
			return RecoverOnFeet(a_actor, a_p.scan.bodies, a_p.w.trunk[0].id, a_p.w.ground, a_p.w.standHeight, a_p.w.yaw);
		}

		void Knock(Puppet& a_p, RE::Actor* a_player)
		{
			a_player->currentProcess->KnockExplosion(a_player, a_player->GetPosition(), 0.0f);  // limp, no push
			a_p.knocked = true;
			a_p.started = NowSeconds();
		}

		// Walking support: trunk springs and their caps follow a factor that wanders between [Player] fDrunkSupportMin and
		// fDrunkSupportMax, a new target every ~fDrunkSwayTime (0.5-1.5x), eased in: it sags, sways and catches itself.
		float Sway(Puppet& a_p)
		{
			const float t = a_p.w.time;
			if (t - a_p.swayStart >= a_p.swayLength) {
				a_p.swayFrom = a_p.sway;
				a_p.swayTo = Random(kDrunkSupportMin, kDrunkSupportMax);
				a_p.swayStart = t;
				a_p.swayLength = std::max(Random(0.5f, 1.5f) * kDrunkSwayTime, 0.05f);
			}
			a_p.sway = a_p.swayFrom + (a_p.swayTo - a_p.swayFrom) * Smooth((t - a_p.swayStart) / a_p.swayLength);
			return a_p.sway;
		}

		void StartDrunk()
		{
			auto*      player = Player();
			const auto refuse = [&](std::string_view a_why) { LogSkipOnce(player, "drunk refused", a_why); };
			if (!player || !player->currentProcess || !player->Get3D()) return refuse("no 3D");
			if (player->IsDead(false)) return refuse("dead");
			if (RE::PowerArmor::ActorInPowerArmor(*player)) return refuse("in power armor");
			if (player->knockState != 0) return refuse("already knocked down");
			if (InLockedAnimation(player)) return refuse("locked animation (furniture, sync, kill move)");
			auto* world = GetWorld(player);
			auto* cell = player->GetParentCell();
			if (!world || !cell) return refuse("no physics world");
			const auto pos = player->GetPosition();
			float      ground = 0.0f;
			if (!CastGround(cell, Scale(Vec3{ pos.x, pos.y, pos.z + 30.0f }, kGameToPhysics), ground)) return refuse("no ground under the player");
			Puppet p{};
			p.world = world;
			p.started = NowSeconds();
			p.w.ground = ground;
			p.scan.formID = player->GetFormID();
			// First person: third person first, the knockdown next frame, so the knockdown's own ragdoll camera takes over
			// as with pushactoraway. (Switching after the knockdown started replaced that camera: it lost the body.)
			auto* camera = RE::PlayerCamera::GetSingleton();
			if (camera && camera->QCameraEquals(RE::CameraState::kFirstPerson)) {
				if (auto third = camera->GetState(RE::CameraState::k3rdPerson)) camera->SetState(third.get());
				REX::INFO("DynamicRagdollSystem: 00000014 drunk: third person"sv);
			} else {
				Knock(p, player);
			}
			g_drunk = std::move(p);
			REX::INFO("DynamicRagdollSystem: 00000014 drunk on (camera heading {:.2f}, player {:.2f}, support {:.2f}-{:.2f})"sv,
				camera ? camera->heading : 0.0f, player->data.angle.z, kDrunkSupportMin, kDrunkSupportMax);
		}
	}

	bool IsDrunk() { return g_drunk.has_value(); }

	// Still on its feet: skip the from-the-floor get-up (RagdollLab Recover).
	bool RecoverOnFeet(RE::Actor* a_actor, const std::vector<BodyRef>& a_bodies, std::uint32_t a_pelvis, float a_ground, float a_standHeight, float a_yaw)
	{
		auto* world = GetWorld(a_actor);
		auto* npWorld = world ? world->m_worldNP.get() : nullptr;
		if (!npWorld || !a_actor->currentProcess) return false;
		const auto id = a_actor->GetFormID();
		Vec3 pelvis{};
		bool found = false;
		std::vector<BonePose> poses;  // where the bodies are now: pinned there while the animation fades in
		{
			WorldWriteLock lock{ npWorld };
			if (!lock.held) return false;
			static bool checked = false;
			for (const auto& ref : a_bodies) {
				const auto* body = FindBody(npWorld->m_bodyManager.m_bodies, ref);
				if (!body) continue;
				const auto pos = BodyPosition(*body);
				if (ref.id.m_value == a_pelvis) {
					pelvis = pos;
					found = true;
				}
				if (ref.bone < 0 || ref.bone >= kMaxRagdollBones) continue;
				const auto q = QuatFrom(body->m_transform.m_rotation);
				if (!checked) {  // the quaternion must turn the body's x axis onto its matrix's first column
					checked = true;
					const auto x = Rotate(q, Vec3{ 1.0f, 0.0f, 0.0f });
					const auto& c = body->m_transform.m_rotation.m_col[0];
					REX::INFO("DynamicRagdollSystem: recover pose check: quaternion error {:.4f} (should be ~0)"sv,
						Length(Sub(x, Vec3{ c.x, c.y, c.z })));
				}
				poses.push_back({ ref.bone, { pos.x, pos.y, pos.z, 0.0f }, { q.x, q.y, q.z, q.w }, { 1.0f, 1.0f, 1.0f, 0.0f }, ref.id.m_value });
			}
		}
		if (!found) return false;
		if (pelvis.z - a_ground < 0.6f * a_standHeight) return false;  // fallen over: vanilla get-up

		a_actor->SetPosition({ pelvis.x * kPhysicsToGame, pelvis.y * kPhysicsToGame, a_ground * kPhysicsToGame }, true);
		a_actor->data.angle = { 0.0f, 0.0f, a_yaw };
		auto* middleHigh = a_actor->currentProcess->middleHigh;
		if (middleHigh && !middleHigh->charController) {  // seen once on an NPC in RagdollLab: walked in place without it
			a_actor->InitHavok();
			REX::INFO("DynamicRagdollSystem: {:08X} recover: controller was missing, rebuilt: {}"sv, id, middleHigh->charController != nullptr);
		}
		// GetUpStart event (OG 0x140F70850, AE 0x140E0E020): resets the character controller from the actor.
		using getUpStart_t = bool (*)(RE::Actor*);
		static REL::Relocation<getUpStart_t> getUpStart{ REL::ID{ 614270, 0, 2236296 } };
		getUpStart(a_actor);
		// End knockdown (task 0xC, OG 0x140E743E0, AE 0x140D35E40): removes the ragdoll, knockState 0. No-op while paralyzed.
		using end_t = void (*)(RE::AIProcess*, RE::Actor*, bool);
		static REL::Relocation<end_t> endKnockdown{ REL::ID{ 1355205, 0, 2232386 } };
		endKnockdown(a_actor->currentProcess, a_actor, false);
		if (a_actor->knockState != 0) {
			REX::INFO("DynamicRagdollSystem: {:08X} recover: knockdown did not end (state {}), vanilla get-up"sv, id, static_cast<int>(a_actor->knockState));
			return false;
		}
		// The graph is still in its ragdoll state: only the get-up animation's end would take it back to locomotion.
		const bool reverted = a_actor->RevertAnimationGraphManager(true);
		{
			// The hit reaction's drive blends the dynamic bodies back into the animation (fHitSoftness, fReactionTime).
			std::scoped_lock lock{ g_mutex };
			g_pending.push_back({ id, {}, 0x7FFFFFFF, 0.0f, false, true, std::move(poses), { pelvis.x, pelvis.y, a_ground }, a_yaw });
		}
		REX::INFO("DynamicRagdollSystem: {:08X} recovered on its feet (graph reset {})"sv, id, reverted);
		return true;
	}

	void StopDrunk(bool a_recover, const char* a_why)
	{
		if (!g_drunk) return;
		auto& p = *g_drunk;
		// Getting up: let go once standing (DrunkFrame). Asked again: let go now, with the vanilla get-up.
		if (a_recover && p.ready && p.w.rise != Rise::kNone && !p.release) {
			p.release = true;
			REX::INFO("DynamicRagdollSystem: 00000014 drunk: released once back on its feet (key again: now)"sv);
			return;
		}
		auto* player = Player();
		if (p.paralyzed) SetParalysis(player, false);  // the engine's get-up follows, unless Recover ends the knockdown
		const bool recovered = a_recover && p.paralyzed && player && p.w.rise == Rise::kNone && Recover(player, p);
		REX::INFO("DynamicRagdollSystem: 00000014 drunk off ({}){}"sv, a_why, recovered ? "" : ", vanilla get-up");
		g_drunk.reset();
	}

	void DrunkFrame(RE::bhkWorld* a_world)
	{
		if (!g_drunkEnabled) return;
		auto* player = Player();
		auto* playerWorld = player ? GetWorld(player) : nullptr;
		if (g_drunk && !playerWorld) return StopDrunk(false, "no physics world");
		if (!playerWorld || playerWorld != a_world) return;  // another world's update this frame
		// Followed, not released (an exterior cell border changed it once): a body really gone ends it in DrunkStep.
		if (g_drunk && g_drunk->world != playerWorld) {
			REX::INFO("DynamicRagdollSystem: 00000014 drunk: physics world changed; following"sv);
			g_drunk->world = playerWorld;
		}
		const bool pressed = Pressed(kDrunkKey);
		if (!g_drunk) {
			if (pressed) StartDrunk();
			return;
		}
		auto& p = *g_drunk;
		if (p.stop) return StopDrunk(false, p.stop);
		if (player->IsDead(false)) return StopDrunk(false, "dead");
		if (RE::PowerArmor::ActorInPowerArmor(*player)) return StopDrunk(false, "power armor");
		if (!player->Get3D()) return StopDrunk(false, "no 3D");
		if (pressed) return StopDrunk(true, p.w.rise == Rise::kNone ? "key" : p.release ? "key again" : "key");
		if (p.release && p.ready && p.w.rise == Rise::kNone) return StopDrunk(true, "back on its feet");
		if (!p.knocked) return Knock(p, player);  // the frame after leaving first person
		if (player->knockState != 0 && !p.paralyzed) {
			SetParalysis(player, true);
			p.paralyzed = true;
		}
		if (player->knockState == 0 && NowSeconds() - p.started > 2.0f) return StopDrunk(false, "knockdown ended");

		// WASD relative to the camera's heading (the player's own angle is frozen in the knockdown); Shift runs.
		Vec3 input{};
		if (InputAllowed()) {
			input.y = static_cast<float>(KeyDown('W') - KeyDown('S'));
			input.x = static_cast<float>(KeyDown('D') - KeyDown('A'));
		}
		const float heading = RE::PlayerCamera::GetSingleton()->heading;
		p.w.moveVel = Length(input) > 0.0f ? Scale(Unit(Rotate(Yaw(heading), input)), KeyDown(VK_SHIFT) ? kDrunkRunSpeed : kDrunkWalkSpeed) : Vec3{};

		// Ground under the pelvis (rays need the read lock: here, not in the step).
		if (p.ready) {
			auto* npWorld = a_world->m_worldNP.get();
			const BodyRef* ref = nullptr;
			for (const auto& b : p.scan.bodies)
				if (b.id.m_value == p.w.trunk[0].id) ref = &b;
			const auto* body = npWorld && ref ? FindBody(npWorld->m_bodyManager.m_bodies, *ref) : nullptr;
			float ground = 0.0f;
			if (body && CastGround(player->GetParentCell(), BodyPosition(*body), ground)) p.w.ground = ground;
		}
	}

	void DrunkStep(RE::bhkWorld* a_world, float a_dt)
	{
		if (!g_drunk || g_drunk->stop || a_dt <= 0.0f || g_drunk->world != a_world) return;
		auto& p = *g_drunk;
		auto* player = Player();
		auto* npWorld = a_world->m_worldNP.get();
		if (!player || !npWorld || player->knockState == 0) return;
		if (!p.ready) {
			if (BuildWalker(p, player, npWorld)) p.ready = true;
			else if (NowSeconds() - p.started > kSetupWait) p.stop = "skeleton not usable (needs a pelvis and two legs)";
			return;
		}
		auto& worldBodies = npWorld->m_bodyManager.m_bodies;
		Ctx   c{ npWorld, p.w, {}, a_dt, -npWorld->m_gravity.z };
		c.turnRate = kDrunkTurnRate;
		for (const auto& body : p.scan.bodies)
			if (const auto* b = FindBody(worldBodies, body)) c.bodies[body.id.m_value] = b;
		auto&      s = p.w;
		const auto missing = [&](std::uint32_t a_id) { return !c.Body(a_id); };
		bool lost = std::any_of(s.trunk.begin(), s.trunk.end(), [&](const Part& a_p) { return missing(a_p.id); });
		for (const auto& f : s.feet) lost |= missing(f.id) || missing(f.calf) || missing(f.thigh);
		MotionState pelvis{};
		if (lost || !c.Read(s.trunk[0].id, pelvis)) {
			p.stop = "body lost";
			return;
		}
		s.time += a_dt;
		if (!GetUp(c)) {  // getting up always at full strength
			c.strength = c.stiffness = Sway(p);
			Walk(c);
		}
	}
}
