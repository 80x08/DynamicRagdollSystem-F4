// RagdollLabSim: Fallout 4's human ragdoll (dumped from the game) in Jolt, driven by the plugin's puppet controller.
// Usage: RagdollLabSim [settings.toml] [ragdoll.json] [plugin.toml]
//   (defaults: RagdollLab.toml, human_ragdoll.json, DynamicRagdollSystem.toml; each toml next to the exe, else in
//   <exe>/../../../../../res; DynamicRagdollSystem.toml must exist)

#include "Death.h"
#include "PluginSettings.h"
#include "Puppet.h"
#include "Sidebar.h"

#include "ConfigIO.h"

#include <raylib.h>
#include <rlgl.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

namespace
{
	Vector3 ToRl(JPH::Vec3 a_v) { return { a_v.GetX(), a_v.GetY(), a_v.GetZ() }; }

	struct SettingsFile
	{
		std::filesystem::path           path;
		bool                            plugin{};  // the plugin's DynamicRagdollSystem.toml: read only, never written
		std::filesystem::file_time_type stamp{};
		std::string                     status;

		std::vector<Lab::Setting> Table() const
		{
			if (plugin) return PluginTable(g_plugin);
			auto table = SimTable(g_sim);
			for (const auto& s : Lab::ConfigTable(Lab::g_config)) table.push_back(s);
			return table;
		}

		// Returns true when values were (re)loaded.
		bool Poll()
		{
			std::error_code ec;
			if (!plugin && !std::filesystem::exists(path, ec)) {
				std::ofstream out{ path };
				out << "# RagdollLab settings. [Sim] is the simulator's own; [Puppet] is the walker.\n";
				Lab::WriteSettings(out, Table());
			}
			const auto now = std::filesystem::last_write_time(path, ec);
			if (ec || now == stamp) return false;
			stamp = now;
			try {
				Lab::ApplySettings(toml::parse(path.string()), Table());
				FixRanges(g_plugin);
				status = "settings loaded: " + path.filename().string();
			} catch (const std::exception& e) {
				status = std::string("settings error, kept old values: ") + e.what();
			}
			std::printf("%s\n", status.c_str());
			return true;
		}
	};
}

// --test: no window. Stand 5 s, walk forward 3 s, turn right 2 s, turn back 2 s, stop 2 s. Exit code 0 if the pelvis
// stayed up, the puppet moved, and the gait stayed sane (feet not both in the air, no high kicks, no feet curled far
// behind, legs never crossed, planted feet not skating).
int Test(World& a_world, Puppet& a_puppet)
{
	const float     dt = 1.0f / 60.0f;
	auto&           bodies = a_world.Bodies();
	const JPH::Vec3 forward(std::sin(a_world.StartYaw()), std::cos(a_world.StartYaw()), 0);
	const JPH::Vec3 right(std::cos(a_world.StartYaw()), -std::sin(a_world.StartYaw()), 0);
	const auto      pos = [&](int a_part) { return JPH::Vec3(bodies.GetPosition(a_world.Body(a_part))); };
	const auto      feetApart = [&](JPH::Vec3 a_right) { return (pos(a_puppet.FootPart(0)) - pos(a_puppet.FootPart(1))).Dot(a_right); };
	const JPH::Vec3 start = pos(0);
	const float     side = feetApart(right) < 0.0f ? -1.0f : 1.0f;  // which way foot 0 sits from foot 1
	const float     footZ[2]{ pos(a_puppet.FootPart(0)).GetZ(), pos(a_puppet.FootPart(1)).GetZ() };
	float           low = 9.0f, kick = 0.0f, behind = 0.0f, slipSum = 0.0f, walked = 0.0f;
	int             swings = 0, lastSwing = -1, bothUp = 0, standSteps = 0, crossed = 0, slipCount = 0;
	for (int i = 0; i < 840; ++i) {
		const float     walk = Lab::g_config.puppet.walkSpeed;
		const JPH::Vec3 move = i < 300 ? JPH::Vec3::sZero() : i < 480 ? forward * walk : i < 600 ? right * walk : i < 720 ? -forward * walk :
		                                                                                                                     JPH::Vec3::sZero();
		a_puppet.Step(a_world, move, dt);
		a_world.Step(dt);
		low = std::min(low, pos(0).GetZ());
		if (i == 479) walked = (pos(0) - start).Dot(forward);
		if (a_puppet.swing >= 0 && a_puppet.swing != lastSwing) {
			++swings;
			if (i < 300) ++standSteps;
		}
		lastSwing = a_puppet.swing;
		if (i <= 30) continue;  // gait checks once the pose has settled
		const JPH::Vec3 facing(std::sin(a_puppet.yaw), std::cos(a_puppet.yaw), 0), facingRight(facing.GetY(), -facing.GetX(), 0);
		crossed += feetApart(facingRight) * side < 0.03f;
		int up = 0;
		for (int f = 0; f < 2; ++f) {
			const int   part = a_puppet.FootPart(f);
			const float lift = pos(part).GetZ() - footZ[f];
			up += lift > 0.05f;
			kick = std::max(kick, lift);
			behind = std::max(behind, -(pos(part) - pos(0)).Dot(facing));
			if (a_puppet.swing != f && lift < 0.03f) {
				auto v = JPH::Vec3(bodies.GetLinearVelocity(a_world.Body(part)));
				v.SetZ(0);
				slipSum += v.Length();
				++slipCount;
			}
		}
		bothUp += up == 2;
		if (i % 60 == 59)
			std::printf("t=%.0fs pelvis z=%.2f  moved %.2f fwd %.2f right  steps %d\n", (i + 1) * dt, pos(0).GetZ(), (pos(0) - start).Dot(forward),
				(pos(0) - start).Dot(right), swings);
	}
	const float slip = slipCount ? slipSum / slipCount : 0.0f;
	const bool  up = low > 0.6f * start.GetZ() && walked > 1.0f && swings >= 4;
	const bool  gait = bothUp < 6 && kick < 0.35f && behind < 0.45f && standSteps == 0 && crossed == 0 && slip < 0.3f;
	std::printf("lowest pelvis %.2f m (start %.2f), walked %.2f m forward, %d steps (%d while standing still)\n", low, start.GetZ(), walked,
		swings, standSteps);
	std::printf("gait: both feet up %d frames, legs crossed %d frames, highest foot %.2f m, foot behind pelvis %.2f m, planted foot "
	            "slides %.2f m/s on average\n",
		bothUp, crossed, kick, behind, slip);

	// Falls. Each time it must go down, get up by itself within 15 s, without kicks or launches, and still be standing
	// 2 s later.
	bool       rose = true, smooth = true;  // smooth: no lunges, kicks or launches while getting up
	const auto fall = [&](const char* a_name, float a_turn, float a_speed, int a_limpFrames) {
		const unsigned  before = a_puppet.getUps;
		const JPH::Vec3 shove = JPH::Vec3(std::sin(a_puppet.yaw + a_turn), std::cos(a_puppet.yaw + a_turn), 0) * a_speed;
		for (int p = 0; p < a_world.PartCount(); ++p)
			if (a_puppet.IsTrunk(p) && pos(p).GetZ() > pos(0).GetZ() + 0.1f) bodies.AddLinearVelocity(a_world.Body(p), shove);
		for (int k = 0; k < a_limpFrames; ++k) a_world.Step(dt);
		bool         fell = false;
		int          i = 0;
		Puppet::Rise last = Puppet::Rise::kNone;
		float        violence = 0.0f, limbs = 0.0f;  // fastest pelvis / fastest body, m/s, once getting up has begun
		JPH::Vec3    lay = JPH::Vec3::sZero();
		std::string  fastest;
		for (; i < 900 && a_puppet.getUps == before; ++i) {
			a_puppet.Step(a_world, JPH::Vec3::sZero(), dt);
			a_world.Step(dt);
			fell |= a_puppet.rise != Puppet::Rise::kNone;
			if (a_puppet.rise != last) {
				if (last == Puppet::Rise::kDown && lay.IsNearZero()) lay = pos(0);
				std::printf("  %.2fs %s (pelvis z %.2f)\n", i * dt, Puppet::RiseName(last = a_puppet.rise), pos(0).GetZ());
			}
			if (a_puppet.rise == Puppet::Rise::kNone || a_puppet.rise == Puppet::Rise::kDown) continue;
			violence = std::max(violence, JPH::Vec3(bodies.GetLinearVelocity(a_world.Body(0))).Length());
			for (int p = 0; p < a_world.PartCount(); ++p) {
				const float v = JPH::Vec3(bodies.GetLinearVelocity(a_world.Body(p))).Length();
				if (v > limbs) {
					limbs = v;
					fastest = a_world.Part(p).name + " while " + Puppet::RiseName(a_puppet.rise);
				}
			}
		}
		JPH::Vec3 moved = pos(0) - lay;
		moved.SetZ(0);
		std::printf("  getting up: pelvis up to %.2f m/s, fastest body %.2f m/s (%s), pelvis moved %.2f m across the floor\n", violence, limbs,
			fastest.c_str(), moved.Length());
		smooth &= violence < 2.0f && limbs < 5.0f && moved.Length() < 1.0f;
		const float gotUp = i * dt;
		for (int k = 0; k < 120; ++k) {
			a_puppet.Step(a_world, JPH::Vec3::sZero(), dt);
			a_world.Step(dt);
		}
		const bool ok = fell && a_puppet.getUps > before && a_puppet.rise == Puppet::Rise::kNone && pos(0).GetZ() > 0.8f * start.GetZ();
		std::printf("%s: %s, %s after %.1f s, pelvis %.2f m 2 s later\n", a_name, fell ? "fell" : "did not fall", a_puppet.getUps > before ? "up" : "NOT up",
			gotUp, pos(0).GetZ());
		rose &= ok;
	};
	// Key 3 catches shoves: limp for 1.5 s after each so it collapses (forward lands on the belly, backward on the back).
	fall("shoved forward", 0.0f, 6.0f, 90);
	fall("shoved backward", 3.14159f, 6.0f, 90);
	fall("shoved left", -1.5708f, 6.0f, 90);
	fall("shoved right", 1.5708f, 6.0f, 90);
	// Key 1: knocked down, gets up.
	a_puppet.KnockDown();
	fall("knocked down (key 1)", 0.0f, 4.0f, 0);
	// Key 4: soft springs. Must stand still on its own, catch a light push with steps, and fall from a hard one (which key
	// 3 catches), get up, and stay up once it has gone soft again.
	a_puppet.weak = true;
	const auto stand = [&](int a_frames, const char* a_what) {
		bool steady = true;
		for (int k = 0; k < a_frames; ++k) {
			a_puppet.Step(a_world, JPH::Vec3::sZero(), dt);
			a_world.Step(dt);
			steady &= a_puppet.rise == Puppet::Rise::kNone;
		}
		std::printf("weak (key 4): %s %s\n", steady ? "stays up" : "FELL", a_what);
		rose &= steady;
	};
	stand(180, "standing still");
	for (int p = 0; p < a_world.PartCount(); ++p)
		if (a_puppet.IsTrunk(p) && pos(p).GetZ() > pos(0).GetZ() + 0.1f)
			bodies.AddLinearVelocity(a_world.Body(p), JPH::Vec3(std::sin(a_puppet.yaw), std::cos(a_puppet.yaw), 0) * 2.0f);
	stand(240, "after a 2 m/s push (caught with steps)");
	fall("weak (key 4), 6 m/s push", 0.0f, 6.0f, 0);
	stand(300, "for 5 s after getting up");
	a_puppet.weak = false;
	std::printf("%s\n", up && gait && rose && smooth ? "PASS" : "FAIL");
	return up && gait && rose && smooth ? 0 : 2;
}

// The sidebar's Save on a temporary copy of the plugin's file: one value changed and saved must read back changed, and
// every other key in the file must keep its value (numbers compared as numbers: Save writes 8 as 8.0).
int SaveTest(const std::filesystem::path& a_file)
{
	const auto copy = std::filesystem::temp_directory_path() / "RagdollLabSim_save_test.toml";
	std::error_code ec;
	std::filesystem::copy_file(a_file, copy, std::filesystem::copy_options::overwrite_existing, ec);
	PluginSettings tuned = g_plugin;
	tuned.kneeBend = tuned.kneeBend == 42.5f ? 43.5f : 42.5f;
	std::string error;
	bool        ok = !ec && Lab::SaveSettings(copy, PluginTable(tuned), error);
	try {
		const auto before = toml::parse(a_file.string()), after = toml::parse(copy.string());
		for (const auto& [section, table] : before.as_table()) {
			if (!table.is_table()) continue;
			for (const auto& [key, value] : table.as_table()) {
				const auto* saved = Lab::FindSetting(after, section.c_str(), key.c_str());
				const bool  changed = section == "DeathStumble" && key == "fDeathKneeBend";
				const auto  num = [](const toml::value& a_v) { return a_v.is_floating() ? a_v.as_floating() : double(a_v.as_integer()); };
				const bool  numeric = (value.is_floating() || value.is_integer()) && saved && (saved->is_floating() || saved->is_integer());
				const bool  same = saved && (numeric ? std::abs(num(*saved) - (changed ? tuned.kneeBend : num(value))) < 1e-6 : *saved == value);
				if (!same) {
					std::printf("save test: [%s] %s differs after saving\n", section.c_str(), key.c_str());
					ok = false;
				}
			}
		}
	} catch (const std::exception& e) {
		error = e.what();
		ok = false;
	}
	std::filesystem::remove(copy, ec);
	std::printf("save test: %s%s\n", ok ? "PASS" : "FAIL ", ok ? "" : error.c_str());
	return ok ? 0 : 2;
}

// Left click's roll (Death::Roll) with set chances: headless-only walk never walks in the sim; a sure walk walks; with no
// walk, a sure stagger staggers, else a sure stumble stumbles, else nothing; a sure head shot skip cancels a head shot.
int RollTest(World& a_world)
{
	const PluginSettings saved = g_plugin;
	int                  chest = 0, head = 0;
	for (int i = 0; i < a_world.PartCount(); ++i) {
		if (a_world.Part(i).name == "Chest") chest = i;
		if (a_world.Part(i).name == "Head") head = i;
	}
	using K = std::optional<Death::Kind>;
	const auto roll = [&](bool a_headlessOnly, float a_walk, float a_stagger, float a_stumble, float a_skip, int a_part) {
		g_plugin.walkHeadlessOnly = a_headlessOnly, g_plugin.walkChance = a_walk, g_plugin.staggerChance = a_stagger;
		g_plugin.stumbleChance = a_stumble, g_plugin.headshotSkipChance = a_skip;
		bool grip;
		return Death::Roll(a_world, a_part, grip);
	};
	const bool ok = roll(true, 1, 0, 0, 0, chest) == K{} && roll(false, 1, 0, 0, 0, chest) == K{ Death::Kind::kWalk } &&
	                roll(false, 0, 1, 0, 0, chest) == K{ Death::Kind::kStagger } && roll(false, 0, 0, 1, 0, chest) == K{ Death::Kind::kStumble } &&
	                roll(false, 0, 0, 0, 0, chest) == K{} && roll(false, 0, 1, 1, 1, head) == K{} &&
	                roll(false, 0, 1, 1, 1, chest) == K{ Death::Kind::kStagger };
	g_plugin = saved;
	std::printf("roll test: %s\n", ok ? "PASS" : "FAIL");
	return ok ? 0 : 2;
}

// Each death (Q walk, W stagger, E stumble): shot in the chest from behind. It must step to catch itself, clutch the
// wound with a hand, not be launched, and be down once it is over (5 s; the walk, which lasts up to 8 s: 10 s). The
// pelvis height at 0.8 s is printed, not checked: the stagger is usually still on its feet then (~0.95 m). The walk must
// also still be up at 1.5 s (it walks at full strength for the first 40%) and take at least 4 steps.
int DeathTest(World& a_world, Death::Kind a_kind)
{
	const float dt = 1.0f / 60.0f;
	auto&       bodies = a_world.Bodies();
	const auto  pos = [&](int a_part) { return JPH::Vec3(bodies.GetPosition(a_world.Body(a_part))); };
	a_world.Reset();
	int chest = 0;
	for (int i = 0; i < a_world.PartCount(); ++i)
		if (a_world.Part(i).name == "Chest") chest = i;
	const float start = pos(0).GetZ();
	Death       death;
	death.Start(a_world, chest, JPH::Vec3(std::sin(a_world.StartYaw()), std::cos(a_world.StartYaw()), 0), 1.0f, a_kind);
	const bool walk = a_kind == Death::Kind::kWalk;
	float      early = 0.0f, walking = 0.0f, fastest = 0.0f, gripFrom = -1.0f, gripBest = 9.0f;
	for (int i = 0; i < (walk ? 600 : 300); ++i) {
		death.Step(a_world, dt);
		a_world.Step(dt);
		if (i == 47) early = pos(0).GetZ();
		if (i == 89) walking = pos(0).GetZ();
		for (int p = 0; p < a_world.PartCount(); ++p) fastest = std::max(fastest, JPH::Vec3(bodies.GetLinearVelocity(a_world.Body(p))).Length());
		if (death.gripHand >= 0 && i < 150) {  // within the stumble (~2.5 s): it staggers on its feet before it drops
			const float d = (pos(death.gripHand) - pos(chest)).Length();
			if (gripFrom < 0.0f) gripFrom = d;
			gripBest = std::min(gripBest, d);
		}
	}
	const bool ok = death.steps >= (walk ? 4u : 2u) && pos(0).GetZ() < 0.6f * start && fastest < 8.0f && gripBest < gripFrom - 0.08f &&
	                (!walk || walking > 0.8f * start);
	std::printf("%s: pelvis %.2f m at 0.8 s, %.2f m at 1.5 s, %.2f m at the end (start %.2f), %u steps, fastest body %.2f m/s, grip hand %.2f -> "
	            "%.2f m from the wound: %s\n",
		Death::KindName(a_kind), early, walking, pos(0).GetZ(), start, death.steps, fastest, gripFrom, gripBest, ok ? "PASS" : "FAIL");
	return ok ? 0 : 2;
}

// Head shot from a_turn (rad from the body's facing): the puppet stands 2 s, then dies. Prints the clutching hand's
// distance to the head and how far it sits behind the chest (trunk frame). a_butt: both hands are first pulled behind
// the buttocks.
int HeadShot(World& a_world, Puppet& a_puppet, float a_turn, bool a_butt = false)
{
	const float dt = 1.0f / 60.0f;
	auto&       bodies = a_world.Bodies();
	const auto  pos = [&](int a_part) { return JPH::Vec3(bodies.GetPosition(a_world.Body(a_part))); };
	const auto  find = [&](const char* a_name) {
		for (int i = 0; i < a_world.PartCount(); ++i)
			if (a_world.Part(i).name == a_name) return i;
		return -1;
	};
	const int head = find("Head"), chest = find("Chest"), lArm = find("LArm_UpperArm"), rArm = find("RArm_UpperArm");
	a_world.Reset();
	a_puppet.Capture(a_world, a_world.StartYaw());
	const JPH::Vec3 facing(std::sin(a_world.StartYaw()), std::cos(a_world.StartYaw()), 0), side(facing.GetY(), -facing.GetX(), 0);
	for (int i = 0; i < 120; ++i) {
		a_puppet.Step(a_world, JPH::Vec3::sZero(), dt);
		for (const auto& [hand, sign] : { std::pair{ find("LArm_Hand"), -1.0f }, std::pair{ find("RArm_Hand"), 1.0f } })
			if (a_butt && i >= 30) {
				const JPH::Vec3 spot = pos(0) - facing * 0.2f + side * (0.06f * sign) - JPH::Vec3(0, 0, 0.08f);
				bodies.SetLinearVelocity(a_world.Body(hand), (spot - pos(hand)) * 8.0f);
			}
		a_world.Step(dt);
	}
	const float yaw = a_world.StartYaw() + a_turn;
	Death       death;
	death.Start(a_world, head, JPH::Vec3(std::sin(yaw), std::cos(yaw), 0));
	float behindMax = -9.0f, best = 9.0f, from = -1.0f;
	for (int i = 0; i < 150; ++i) {
		death.Step(a_world, dt);
		a_world.Step(dt);
		if (death.gripHand < 0) continue;
		const JPH::Vec3 up = (pos(head) - pos(0)).Normalized(), right = (pos(rArm) - pos(lArm)).Normalized();
		const JPH::Vec3 forward = up.Cross(right).Normalized();
		const float     behind = -(pos(death.gripHand) - pos(chest)).Dot(forward);
		const float     d = (pos(death.gripHand) - pos(head)).Length();
		if (from < 0.0f) from = d;
		behindMax = std::max(behindMax, behind);
		best = std::min(best, d);
	}
	const bool ok = best < 0.25f;  // resting on the head is ~0.17 (head + hand radius); a snagged hand stays ~0.6 away
	std::printf("head shot%s from %+.0f deg: hand %.2f -> %.2f m from the head, at most %.2f m behind the chest: %s\n",
		a_butt ? " (hands behind the buttocks)" : "", a_turn * 57.3f, from, best, behindMax, ok ? "PASS" : "FAIL");
	return ok ? 0 : 2;
}

// Arm and torso shots: the puppet stands 2 s (a_walk: then walks 1.5 s) and is shot in a_part along a_turn (as
// StumbleShot). Over the stumble: which hand clutches, how close it gets to the wound (wrist to the wounded body's origin;
// the gap between their capsules; the hand's surface to the wound's joint, the shoulder for an upper arm), and the time
// each hand spends
// behind the back (wrist over a chest radius behind the chest's centre, trunk frame as HeadShot); for the free hand also
// how fast it moves in the chest's frame while back there (near 0 = held rigid behind the body). An arm shot must be
// clutched by the other hand, whose surface must come within kArmReach of the shoulder joint: clutching the front of the
// shoulder it is 0.16-0.22 (the joint sits inside the chest's capsule); snagged on the belly on the way, 0.26-0.55.
constexpr float kArmReach = 0.25f;
int ArmShot(World& a_world, Puppet& a_puppet, const char* a_part, float a_turn, const char* a_from, bool a_walk)
{
	const float dt = 1.0f / 60.0f;
	auto&       bodies = a_world.Bodies();
	const auto  pos = [&](int a_p) { return JPH::Vec3(bodies.GetPosition(a_world.Body(a_p))); };
	const auto  find = [&](std::string a_name) {
		for (int i = 0; i < a_world.PartCount(); ++i)
			if (a_world.Part(i).name == a_name) return i;
		return -1;
	};
	const int wound = find(a_part), head = find("Head"), chest = find("Chest"), lArm = find("LArm_UpperArm"), rArm = find("RArm_UpperArm");
	a_world.Reset();
	a_puppet.Capture(a_world, a_world.StartYaw());
	const JPH::Vec3 facing(std::sin(a_world.StartYaw()), std::cos(a_world.StartYaw()), 0);
	for (int i = 0; i < (a_walk ? 210 : 120); ++i)
		a_puppet.Step(a_world, i >= 120 ? facing * Lab::g_config.puppet.walkSpeed : JPH::Vec3::sZero(), dt), a_world.Step(dt);
	const float yaw = a_world.StartYaw() + a_turn;
	Death       death;
	death.Start(a_world, wound, JPH::Vec3(std::sin(yaw), std::cos(yaw), 0));
	const float back = a_world.Part(chest).radius;
	// Surface gap between two parts' capsules (0 = touching), sampled along both segments.
	const auto gap = [&](int a_p, int a_q) {
		const auto at = [&](int a_i, float a_t) {
			const auto& part = a_world.Part(a_i);
			return JPH::Vec3(bodies.GetWorldTransform(a_world.Body(a_i)) * (part.a + (part.b - part.a) * a_t));
		};
		float d = 9.0f;
		for (int s = 0; s <= 8; ++s)
			for (int t = 0; t <= 8; ++t) d = std::min(d, (at(a_p, s / 8.0f) - at(a_q, t / 8.0f)).Length());
		return std::max(0.0f, d - a_world.Part(a_p).radius - a_world.Part(a_q).radius);
	};
	const auto toJoint = [&](int a_p, int a_q) {  // a_p's capsule surface to a_q's origin (its joint)
		const auto& part = a_world.Part(a_p);
		const auto  m = bodies.GetWorldTransform(a_world.Body(a_p));
		float       d = 9.0f;
		for (int s = 0; s <= 8; ++s) d = std::min(d, (JPH::Vec3(m * (part.a + (part.b - part.a) * (s / 8.0f))) - pos(a_q)).Length());
		return std::max(0.0f, d - part.radius);
	};
	float       best = 9.0f, touch = 9.0f, joint = 9.0f, gripBehind = 0.0f, freeBehind = 0.0f, freeMoved = 0.0f;
	JPH::Vec3   last = JPH::Vec3::sZero();
	for (int i = 0; i < 240 && (i == 0 || death.Active()); ++i) {
		death.Step(a_world, dt);
		a_world.Step(dt);
		if (death.gripHand < 0) continue;
		const int       free = find(a_world.Part(death.gripHand).name[0] == 'L' ? "RArm_Hand" : "LArm_Hand");
		const JPH::Vec3 up = (pos(head) - pos(0)).Normalized(), right = (pos(rArm) - pos(lArm)).Normalized();
		const JPH::Vec3 forward = up.Cross(right).Normalized();
		best = std::min(best, (pos(death.gripHand) - pos(wound)).Length());
		touch = std::min(touch, gap(death.gripHand, wound));
		joint = std::min(joint, toJoint(death.gripHand, wound));
		gripBehind += dt * (-(pos(death.gripHand) - pos(chest)).Dot(forward) > back);
		const JPH::Vec3 local = JPH::Quat(bodies.GetRotation(a_world.Body(chest))).Conjugated() * (pos(free) - pos(chest));
		if (-(pos(free) - pos(chest)).Dot(forward) > back) {
			freeBehind += dt;
			if (!last.IsNearZero()) freeMoved += (local - last).Length();
		}
		last = local;
	}
	const bool arm = a_part[1] == 'A';  // LArm_* / RArm_*
	const bool other = death.gripHand >= 0 && a_world.Part(death.gripHand).name[0] != a_part[0];
	const bool ok = !arm || (other && joint < kArmReach);
	std::printf("%-14s from %-5s%s: %s grips, %.2f m from the wound at closest (gap %.2f, hand to its joint %.2f), behind the back %.2f s; free hand behind %.2f s, moving %.2f m/s "
	            "there%s\n",
		a_part, a_from, a_walk ? " walking" : "", death.gripHand >= 0 ? a_world.Part(death.gripHand).name.c_str() : "no hand", best, touch, joint, gripBehind,
		freeBehind, freeBehind > 0.0f ? freeMoved / freeBehind : 0.0f, arm ? (ok ? ": PASS" : ": FAIL") : "");
	return ok ? 0 : 2;
}

// Stumble shape, summed over shots. Steps are read off the feet themselves (lift-off to touch-down while the pelvis is
// still up), not from what the controller says it is doing.
struct StumbleScore
{
	int   shots{}, steps{}, along{}, against{}, offLine{}, extended{}, crossed{}, lifted{}, dropped{}, slowedSteps{}, split{}, leanSteps{};
	float turn{}, travel{}, length{}, longest{}, first{}, firstLift{}, slowed{}, slide{}, drop{}, endHeight{}, spread{}, limpTilt{}, still{}, kneel{}, jump{}, fallAlong{}, snap{}, bend{};
	void  Print(const char* a_name) const
	{
		const float n = static_cast<float>(std::max(shots, 1)), k = static_cast<float>(std::max(steps, 1));
		std::printf("%-20s: %4.1f steps (%4.1f along, %3.1f against, %3.1f off), step %.2f mean %.2f max %.2f first, first lift %.2f s, "
		            "speed gone at touchdown %3.0f%%, slide %.2f, pelvis %+.2f along, half height %.2f s (%d/%d), limp at %.2f m, turn %3.0f, "
		            "stiff %4.1f, crossed %4.1f, giving way: feet %.2f apart, split %4.1f\n",
			a_name, steps / n, along / n, against / n, offLine / n, length / k, longest, lifted ? first / lifted : 0.0f, lifted ? firstLift / lifted : 0.0f,
			slowedSteps ? 100.0f * slowed / slowedSteps : 0.0f, slide / n, travel / n, dropped ? drop / dropped : 0.0f, dropped, shots, endHeight / n,
			turn / n, extended / n, crossed / n, spread / n, split / n);
		std::printf("%-20s  limp at %.2f m tilted %3.0f deg, still upright %.2f s, static low %.2f s, largest switch %.2f, steps leaning >30 deg "
		            "%.1f, fall along travel %+.2f\n",
			"", endHeight / n, limpTilt / n, still / n, kneel / n, jump, leanSteps / n, fallAlong / n);
		std::printf("%-20s  hit part up to %.2f m/s faster than the pelvis, trunk bent %3.0f deg within 0.3 s\n", "", snap / n, bend / n);
	}
	void Add(const StumbleScore& a)
	{
		shots += a.shots, steps += a.steps, along += a.along, against += a.against, offLine += a.offLine, extended += a.extended;
		crossed += a.crossed, lifted += a.lifted, dropped += a.dropped, slowedSteps += a.slowedSteps, turn += a.turn, travel += a.travel;
		length += a.length, longest = std::max(longest, a.longest), first += a.first, firstLift += a.firstLift, slowed += a.slowed;
		slide += a.slide, drop += a.drop, endHeight += a.endHeight, spread += a.spread, split += a.split;
		limpTilt += a.limpTilt, still += a.still, kneel += a.kneel, jump = std::max(jump, a.jump), leanSteps += a.leanSteps, fallAlong += a.fallAlong;
		snap += a.snap, bend += a.bend;
	}
};

// The puppet stands 2 s (a_walk: then walks 1.5 s), then is shot in a_part along a_turn (rad from its facing; 0 = in the
// back, so it is pushed forward) with the plugin's push scale a_scale, and runs 4 s. Per step (a foot off the floor and
// back down at least 5 cm away, while the pelvis is above half its standing height): its length, whether it went along
// the stagger line, against it, or off to the side (stepping round in a circle), and how much of the pelvis's speed
// along the line was gone at touchdown (of the most it had since the shot). Also: when the first foot lifted, how far
// planted feet slid (shuffling), how far the pelvis travelled, when it dropped below half height, its height when the
// stumble ended, the body turning, a stiff leg stuck out while up (foot in the air, knee nearly straight, foot over
// 0.35 m from the hip), tangled legs, and once the legs give way: how far apart the feet get along the line, and frames
// with a near-straight leg whose foot is over 0.5 m from under its hip while the pelvis is below 0.7 of standing (a split).
// How it falls: pelvis height and trunk tilt when it goes limp, the longest time standing nearly still (pelvis under
// 0.2 m/s) upright (above 0.7 of standing) or low (0.3-0.7), the largest one-frame change in the stumble's strength or
// leg support (going limp included), steps taken with the trunk leaning over 30 degrees, and how well the trunk's
// lean points along the travel line as the pelvis passes half height (1 = falling the way it was going).
void StumbleShot(World& a_world, Puppet& a_puppet, const char* a_part, float a_turn, bool a_walk, float a_scale, StumbleScore& a_score)
{
	const float dt = 1.0f / 60.0f;
	auto&       bodies = a_world.Bodies();
	const auto  pos = [&](int a_p) { return JPH::Vec3(bodies.GetPosition(a_world.Body(a_p))); };
	const auto  flat = [](JPH::Vec3 a_v) { return JPH::Vec3(a_v.GetX(), a_v.GetY(), 0); };
	const auto  find = [&](std::string a_name) {
		for (int i = 0; i < a_world.PartCount(); ++i)
			if (a_world.Part(i).name == a_name) return i;
		return -1;
	};
	a_world.Reset();
	a_puppet.Capture(a_world, a_world.StartYaw());
	const JPH::Vec3 facing(std::sin(a_world.StartYaw()), std::cos(a_world.StartYaw()), 0);
	for (int i = 0; i < (a_walk ? 210 : 120); ++i)
		a_puppet.Step(a_world, i >= 120 ? facing * Lab::g_config.puppet.walkSpeed : JPH::Vec3::sZero(), dt), a_world.Step(dt);
	const float     yaw = a_world.StartYaw() + a_turn;
	const JPH::Vec3 push(std::sin(yaw), std::cos(yaw), 0);
	const int       thigh[2]{ find("LLeg_Thigh"), find("RLeg_Thigh") }, calf[2]{ find("LLeg_Calf"), find("RLeg_Calf") };
	const int       footPart[2]{ find("LLeg_Foot"), find("RLeg_Foot") };
	// Body origins sit on the joints (thigh = hip, calf = knee, foot = ankle): the true knee angle.
	const auto knee = [&](int a_leg) {
		const float c = (pos(calf[a_leg]) - pos(thigh[a_leg])).Normalized().Dot((pos(footPart[a_leg]) - pos(calf[a_leg])).Normalized());
		return std::acos(std::clamp(c, -1.0f, 1.0f)) * 57.3f;
	};
	const auto heading = [&] {
		const JPH::Vec3 r = flat(pos(thigh[1]) - pos(thigh[0]));
		return std::atan2(r.GetY(), r.GetX()) * 57.3f;
	};
	Death death;
	death.Start(a_world, find(a_part), push, a_scale);
	const JPH::Vec3 start = pos(0);
	const float     floor0 = std::min(pos(footPart[0]).GetZ(), pos(footPart[1]).GetZ()), standZ = start.GetZ();
	const int       chest = find("Chest"), hit = find(a_part);
	const JPH::Vec3 trunk0 = (pos(chest) - pos(0)).Normalized();
	float           lastHeading = heading(), turn = 0.0f, peak = 0.0f, liftTilt[2]{}, stillRun = 0.0f, lowRun = 0.0f, lastLegs = 1.0f, lastStrength = 1.0f;
	bool            up[2]{};
	JPH::Vec3       from[2]{}, lastFoot[2]{ pos(footPart[0]), pos(footPart[1]) };
	StumbleScore    s;
	s.shots = 1;
	std::string log;
	for (int i = 0; i < 300; ++i) {
		const bool active = death.Active();
		death.Step(a_world, dt);
		a_world.Step(dt);
		const float t = (i + 1) * dt, pelvisZ = pos(0).GetZ();
		const bool  standing = pelvisZ > 0.5f * standZ;
		const JPH::Vec3 trunkUp = (pos(chest) - pos(0)).Normalized();
		if (t <= 0.3f) {  // the hit: how hard the hit part is thrown relative to the hips, and how far the trunk bends
			s.snap = std::max(s.snap, JPH::Vec3(bodies.GetLinearVelocity(a_world.Body(hit)) - bodies.GetLinearVelocity(a_world.Body(0))).Length());
			s.bend = std::max(s.bend, std::acos(std::clamp(trunkUp.Dot(trunk0), -1.0f, 1.0f)) * 57.3f);
		}
		const float     tilt = std::acos(std::clamp(trunkUp.GetZ(), -1.0f, 1.0f)) * 57.3f;
		const JPH::Vec3 travelLine = death.Line().IsNearZero() ? push : death.Line();
		if (!standing && !s.dropped) s.dropped = 1, s.drop = t, s.fallAlong = flat(trunkUp).NormalizedOr(JPH::Vec3::sZero()).Dot(travelLine);
		if (active && !death.Active()) s.endHeight = pelvisZ, s.limpTilt = tilt, s.jump = std::max({ s.jump, lastLegs, lastStrength });
		if (death.Active()) {
			const bool slow = JPH::Vec3(bodies.GetLinearVelocity(a_world.Body(0))).Length() < 0.2f && t > 0.3f;
			stillRun = slow && pelvisZ > 0.7f * standZ ? stillRun + dt : 0.0f;
			lowRun = slow && pelvisZ <= 0.7f * standZ && pelvisZ > 0.3f * standZ ? lowRun + dt : 0.0f;
			s.still = std::max(s.still, stillRun), s.kneel = std::max(s.kneel, lowRun);
			s.jump = std::max({ s.jump, std::abs(death.legs - lastLegs), std::abs(death.strength - lastStrength) });
			lastLegs = death.legs, lastStrength = death.strength;
		}
		if (!standing) continue;
		const float h = heading();
		turn += std::remainder(h - lastHeading, 360.0f);  // net, while upright (lying, the thighs swing about)
		lastHeading = h;
		const JPH::Vec3 line = death.Line().IsNearZero() ? push : death.Line();  // the way it staggers: the push, plus the walk
		const float     speed = flat(JPH::Vec3(bodies.GetLinearVelocity(a_world.Body(0)))).Dot(line);
		peak = std::max(peak, speed);
		{  // legs tangled: the left foot not clearly left of the right one, across the hips, and the feet close front to back
			const JPH::Vec3 right = flat(pos(thigh[1]) - pos(thigh[0])).Normalized(), ahead(right.GetY(), -right.GetX(), 0);
			const JPH::Vec3 apart = pos(footPart[1]) - pos(footPart[0]);
			s.crossed += apart.Dot(right) < 0.05f && std::abs(apart.Dot(ahead)) < 0.3f;
			if (death.GivingWay()) s.spread = std::max(s.spread, std::abs(apart.Dot(line)));  // feet spreading along the line as the legs go
		}
		for (int leg = 0; leg < 2; ++leg) {
			const JPH::Vec3 f = pos(footPart[leg]);
			const float     lift = f.GetZ() - floor0;
			s.extended += lift > 0.08f && knee(leg) < 12.0f && flat(f - pos(thigh[leg])).Length() > 0.35f;
			s.split += pelvisZ < 0.7f * standZ && knee(leg) < 15.0f && flat(f - pos(thigh[leg])).Length() > 0.5f;
			if (!up[leg] && lift > 0.04f) {
				up[leg] = true;
				from[leg] = f;
				liftTilt[leg] = tilt;
				if (!s.lifted) s.lifted = 1, s.firstLift = t;
			} else if (up[leg] && lift < 0.02f) {
				up[leg] = false;
				const JPH::Vec3 moved = flat(f - from[leg]);
				const float     along = moved.Dot(line), side = std::abs(moved.Dot(JPH::Vec3(line.GetY(), -line.GetX(), 0)));
				if (moved.Length() >= 0.05f) {  // shorter: lifted and put back, not a step
					if (!s.steps) s.first = moved.Length();
					++s.steps;
					s.leanSteps += liftTilt[leg] > 30.0f;
					s.along += along > side;
					s.against += along < -0.05f;
					s.offLine += side > std::abs(along);
					s.length += moved.Length();
					s.longest = std::max(s.longest, moved.Length());
					if (peak > 0.2f) s.slowed += 1.0f - speed / peak, ++s.slowedSteps;
					log += TextFormat("%s%c %+.0f/%.0f cm @%.2f", log.empty() ? "" : ", ", leg ? 'R' : 'L', 100 * along, 100 * side, t);
				}
			} else if (!up[leg]) {
				s.slide += flat(f - lastFoot[leg]).Length();
			}
			lastFoot[leg] = f;
		}
	}
	if (!s.dropped) s.drop = 4.0f;
	s.turn = std::abs(turn);
	s.travel = (pos(0) - start).Dot(push);
	std::printf("  %-10s %-8s x%.1f %+4.0f deg: %d steps [%s], first lift %.2f s, slide %.2f m, pelvis %+.2f m along, half height %.2f s, "
	            "%.2f m when limp, turned %+4.0f, stiff %2d, crossed %2d, giving way: feet %.2f apart, split %2d\n",
		a_part, a_walk ? "walking" : "standing", a_scale, a_turn * 57.3f, s.steps, log.c_str(), s.firstLift, s.slide, s.travel, s.drop, s.endHeight,
		turn, s.extended, s.crossed, s.spread, s.split);
	a_score.Add(s);
}

// Killing-shot push scale for a click and --frames: the plugin scales its push by damage / 25, so a typical rifle hit in
// the game is about 1.2-2 times the plugin's default (--test and --stumble keep 1 so their numbers stay comparable).
constexpr float kShotScale = 1.5f;

// Floor around a_focus and the ragdoll, coloured by what the controllers are doing (inside BeginMode3D).
void DrawScene(World& a_world, const Puppet& a_puppet, const Death& a_death, bool a_puppeting, JPH::Vec3 a_focus)
{
	const float cx = std::round(a_focus.GetX()), cy = std::round(a_focus.GetY());
	DrawTriangle3D({ cx - 20, cy - 20, 0 }, { cx + 20, cy - 20, 0 }, { cx + 20, cy + 20, 0 }, { 52, 56, 64, 255 });
	DrawTriangle3D({ cx - 20, cy - 20, 0 }, { cx + 20, cy + 20, 0 }, { cx - 20, cy + 20, 0 }, { 52, 56, 64, 255 });
	for (int i = -20; i <= 20; ++i) {
		DrawLine3D({ cx + i, cy - 20, 0.001f }, { cx + i, cy + 20, 0.001f }, { 70, 75, 85, 255 });
		DrawLine3D({ cx - 20, cy + i, 0.001f }, { cx + 20, cy + i, 0.001f }, { 70, 75, 85, 255 });
	}
	for (int i = 0; i < a_world.PartCount(); ++i) {
		const auto& part = a_world.Part(i);
		const auto  t = a_world.Bodies().GetWorldTransform(a_world.Body(i));
		const auto  a = ToRl(JPH::Vec3(t * part.a)), b = ToRl(JPH::Vec3(t * part.b));
		Color       color = { 120, 170, 230, 255 };                                                 // legs / arms
		if (a_puppeting && a_puppet.IsTrunk(i)) color = { 235, 160, 80, 255 };                     // held up
		if (a_puppeting && a_puppet.swing >= 0 && a_puppet.FootPart(a_puppet.swing) == i) color = { 235, 80, 80, 255 };  // stepping
		if (a_death.Active() && a_death.stepFoot == i) color = { 235, 80, 80, 255 };
		if (a_death.hitPart == i) color = { 170, 30, 30, 255 };                                     // shot
		if (a_death.gripHand == i) color = { 110, 210, 110, 255 };                                  // clutching the wound
		DrawCapsule(a, b, part.radius, 10, 4, color);
		DrawCapsuleWires(a, b, part.radius, 10, 4, Fade(BLACK, 0.25f));
	}
	if (a_puppeting) {
		DrawSphere(ToRl(a_puppet.base), 0.04f, YELLOW);
		for (const auto& s : a_puppet.spot) DrawCircle3D(ToRl(s - JPH::Vec3(0, 0, s.GetZ() - 0.002f)), 0.08f, { 0, 0, 1 }, 0, SKYBLUE);
	}
}

// --frames: contact sheets of a few death stumbles seen side on to the shot, saved as <scenario>.png in the working
// folder, to check the poses by eye: a frame every 0.1 s for 3.6 s, and for the hit_* sheets every 2 steps (0.033 s) for
// 0.4 s. The puppet stands 2 s (walking ones then walk 1.5 s) and is shot in a_part along a_turn (rad from its facing;
// 0 = in the back) at kShotScale.
void Frames(World& a_world, Puppet& a_puppet)
{
	const float dt = 1.0f / 60.0f;
	const int   w = 320, h = 240, cols = 6;
	SetConfigFlags(FLAG_WINDOW_HIDDEN | FLAG_MSAA_4X_HINT);
	InitWindow(w, h, "RagdollLab Sim");
	if (!IsWindowReady()) {  // no display to render with (e.g. no monitor attached)
		std::printf("--frames: cannot open a window to render with\n");
		return;
	}
	const RenderTexture2D target = LoadRenderTexture(w, h);
	const struct { const char* name; const char* part; float turn; bool walk; int every, frames; bool front{}; } scenes[]{
		{ "front_chest", "Chest", 3.14159f, false, 6, 36 }, { "side_chest", "Chest", 1.5708f, false, 6, 36 },
		{ "back_chest", "Chest", 0.0f, false, 6, 36 }, { "walking_front_chest", "Chest", 3.14159f, true, 6, 36 },
		{ "walking_side_thigh", "LLeg_Thigh", -1.5708f, true, 6, 36 }, { "front_thigh", "LLeg_Thigh", 3.14159f, false, 6, 36 },
		{ "side_thigh", "LLeg_Thigh", 1.5708f, false, 6, 36 }, { "hit_chest", "Chest", 3.14159f, false, 2, 12 },
		{ "hit_head", "Head", 3.14159f, false, 2, 12 }, { "hit_shoulder", "RArm_UpperArm", 3.14159f, false, 2, 12 },
		{ "hit_thigh", "LLeg_Thigh", 1.5708f, false, 2, 12 }, { "facing_shoulder", "RArm_UpperArm", 3.14159f, false, 6, 36, true },
		{ "facing_chest", "Chest", 3.14159f, false, 6, 36, true },
	};
	for (const auto& scene : scenes) {
		a_world.Reset();
		a_puppet.Capture(a_world, a_world.StartYaw());
		const JPH::Vec3 facing(std::sin(a_world.StartYaw()), std::cos(a_world.StartYaw()), 0);
		for (int i = 0; i < (scene.walk ? 210 : 120); ++i)
			a_puppet.Step(a_world, i >= 120 ? facing * Lab::g_config.puppet.walkSpeed : JPH::Vec3::sZero(), dt), a_world.Step(dt);
		const float     yaw = a_world.StartYaw() + scene.turn;
		const JPH::Vec3 push(std::sin(yaw), std::cos(yaw), 0), across(push.GetY(), -push.GetX(), 0);
		int             part = 0;
		for (int i = 0; i < a_world.PartCount(); ++i)
			if (a_world.Part(i).name == scene.part) part = i;
		Death death;
		death.Start(a_world, part, push, kShotScale);
		const JPH::Vec3 focus = JPH::Vec3(a_world.Bodies().GetPosition(a_world.Body(0))) + push * 0.6f;  // halfway along its usual travel
		const Camera3D  camera{ ToRl(focus + (scene.front ? facing * 2.5f : across * 4.0f)), ToRl(focus), { 0, 0, 1 }, 45.0f, CAMERA_PERSPECTIVE };
		Image           sheet = GenImageColor(w * cols, h * ((scene.frames + cols - 1) / cols), BLACK);
		for (int i = 0; i < scene.every * scene.frames; ++i) {
			if (i % scene.every == 0) {
				BeginTextureMode(target);
				ClearBackground({ 30, 32, 38, 255 });
				BeginMode3D(camera);
				DrawScene(a_world, a_puppet, death, false, focus);
				EndMode3D();
				DrawText(TextFormat("%.1f s  steps %u%s", i * dt, death.steps, death.Active() ? "" : "  limp"), 6, 6, 20, RAYWHITE);
				EndTextureMode();
				Image frame = LoadImageFromTexture(target.texture);
				ImageFlipVertical(&frame);
				const int k = i / scene.every;
				ImageDraw(&sheet, frame, { 0, 0, float(w), float(h) }, { float(k % cols * w), float(k / cols * h), float(w), float(h) }, WHITE);
				UnloadImage(frame);
			}
			death.Step(a_world, dt);
			a_world.Step(dt);
		}
		ExportImage(sheet, TextFormat("%s.png", scene.name));
		UnloadImage(sheet);
	}
	UnloadRenderTexture(target);
	CloseWindow();
}

int main(int argc, char** argv)
{
	std::vector<std::string> args(argv + 1, argv + argc);
	const bool               stumble = std::erase(args, std::string("--stumble")) > 0;  // step diagnostics only
	const bool               headOnly = std::erase(args, std::string("--head")) > 0;  // --test's head shots only
	const bool               armsOnly = std::erase(args, std::string("--arms")) > 0;  // --test's arm and chest shots only
	const bool               test = std::erase(args, std::string("--test")) > 0 || headOnly || armsOnly;
	const bool               frames = std::erase(args, std::string("--frames")) > 0;  // contact sheets of a few stumbles
	// Default settings: next to the exe, else the repo's res folder (from the exe in build/windows/x64/<mode>/), whatever
	// the working folder. RagdollLab.toml is written to res with defaults when it is in neither.
	const auto exe = std::filesystem::absolute(argv[0]).parent_path(), res = (exe / "../../../../../res").lexically_normal();
	const auto find = [&](const char* a_name) {
		std::error_code ec;
		return std::filesystem::is_regular_file(exe / a_name, ec) ? exe / a_name : res / a_name;
	};
	SettingsFile settings{ args.size() > 0 ? args[0] : find("RagdollLab.toml") };
	SettingsFile pluginSettings{ args.size() > 2 ? args[2] : find("DynamicRagdollSystem.toml"), true };
	if (std::error_code ec; !std::filesystem::is_regular_file(pluginSettings.path, ec)) {
		std::printf("Cannot find the plugin's settings %s (looked next to the exe and in %s; or pass its path as the third argument)\n",
			std::filesystem::absolute(pluginSettings.path, ec).string().c_str(), res.string().c_str());
		return 1;
	}
	settings.Poll();
	pluginSettings.Poll();

	World       world;
	std::string error;
	if (!world.Load(args.size() > 1 ? args[1] : "human_ragdoll.json", error)) {
		std::printf("Cannot load the ragdoll: %s\n", error.c_str());
		return 1;
	}
	Puppet puppet;
	Death  death;
	bool   puppeting = puppet.Capture(world, world.StartYaw());
	if (stumble || test || frames) Death::rng.seed(1);
	if (stumble) {
		// Shot from the front (body driven backward), from behind, from its left, from its right.
		const float turns[4]{ 3.14159f, 0.0f, 1.5708f, -1.5708f };
		const char* dirs[4]{ "front", "back", "left", "right" };
		const char* parts[3]{ "Chest", "Head", "LLeg_Thigh" };
		StumbleScore byDir[2][4], byPart[2][3], all;
		for (int round = 0; round < 2; ++round)
			for (int walk = 0; walk < 2; ++walk)
				for (int p = 0; p < 3; ++p)
					for (int d = 0; d < 4; ++d) {
						StumbleScore one;
						StumbleShot(world, puppet, parts[p], turns[d], walk == 1, 1.0f, one);
						byDir[walk][d].Add(one), byPart[walk][p].Add(one), all.Add(one);
					}
		StumbleScore strength[5][3];  // chest shot standing, from the front, back and left: weak to very strong
		const float  scales[5]{ 0.5f, 1.0f, 1.5f, 2.0f, 3.0f };
		for (int round = 0; round < 2; ++round)
			for (int k = 0; k < 5; ++k)
				for (int d = 0; d < 3; ++d) StumbleShot(world, puppet, "Chest", turns[d], false, scales[k], strength[k][d]);
		for (int walk = 0; walk < 2; ++walk) {
			for (int d = 0; d < 4; ++d) byDir[walk][d].Print(TextFormat("%s %s", walk ? "walking" : "standing", dirs[d]));
			for (int p = 0; p < 3; ++p) byPart[walk][p].Print(TextFormat("%s %s", walk ? "walking" : "standing", parts[p]));
		}
		for (int k = 0; k < 5; ++k)
			for (int d = 0; d < 3; ++d) strength[k][d].Print(TextFormat("%s chest x%.1f", dirs[d], scales[k]));
		all.Print("all x1");
		return 0;
	}
	if (frames) {
		if (puppeting) Frames(world, puppet);
		return puppeting ? 0 : 1;
	}
	if (test) {
		if (!puppeting) return 1;
		const int walker = headOnly || armsOnly ? 0 : Test(world, puppet);
		int       heads = 0, arms = 0;
		for (const bool butt : { false, true })
			for (const float turn : { 0.0f, 3.14159f, 1.5708f, -1.5708f })
				if (!armsOnly) heads = std::max(heads, HeadShot(world, puppet, turn, butt));
		int stagger = 0;
		for (const auto kind : { Death::Kind::kStagger, Death::Kind::kWalk, Death::Kind::kStumble })
			if (!headOnly && !armsOnly) stagger = std::max(stagger, DeathTest(world, kind));
		// Shoulders from the four sides (standing, and from the front while walking); the chest for the free hand.
		const float turns[4]{ 3.14159f, 0.0f, 1.5708f, -1.5708f };
		const char* dirs[4]{ "front", "back", "left", "right" };
		for (const char* part : { "LArm_UpperArm", "RArm_UpperArm", "Chest" })
			for (int d = 0; d < 4 && !headOnly; ++d) arms = std::max(arms, ArmShot(world, puppet, part, turns[d], dirs[d], false));
		for (const char* part : { "LArm_UpperArm", "RArm_UpperArm" })
			if (!headOnly) arms = std::max(arms, ArmShot(world, puppet, part, turns[0], dirs[0], true));
		const int result = std::max({ walker, stagger, heads, arms, SaveTest(pluginSettings.path), RollTest(world) });
		std::printf("%s\n", result ? "FAIL" : "PASS");
		return result;
	}

	SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
	InitWindow(1280, 800, "RagdollLab Sim");
	SetTargetFPS(60);

	Camera3D camera{};
	camera.up = { 0, 0, 1 };
	camera.fovy = 50.0f;
	camera.projection = CAMERA_PERSPECTIVE;
	float      heading = world.StartYaw() + 2.4f, pitch = 0.35f, distance = 3.5f;  // camera heading: game convention
	JPH::Vec3  focus(0, 0, 1);
	const float kStep = 1.0f / 60.0f;
	float       accumulator = 0.0f, pollTimer = 0.0f;
	bool        paused = false;
	Sidebar     sidebar;  // the plugin's settings the sim uses, tunable; Save writes them to its file, Reset goes back to it
	sidebar.Load();
	PluginSettings fileValues = g_plugin;  // the plugin file's values as last loaded or saved (Reset)

	while (!WindowShouldClose()) {
		const float frame = std::min(GetFrameTime(), 0.1f);
		if ((pollTimer += frame) > 0.5f) {
			pollTimer = 0.0f;
			if (settings.Poll()) world.ApplySettings();
			if (pluginSettings.Poll()) sidebar.dirty = false, fileValues = g_plugin;  // changed on disk: replaces unsaved values
		}

		// Input. The 3D view is the window left of the sidebar. While a value is typed in the sidebar, the keys are its.
		const bool typing = sidebar.Typing();
		const auto pressed = [&](int a_key) { return !typing && IsKeyPressed(a_key); };
		const auto down = [&](int a_key) { return !typing && IsKeyDown(a_key); };
		const int  viewW = std::max(1, GetScreenWidth() - Sidebar::kWidth), viewH = std::max(1, GetScreenHeight());
		const bool overSidebar = sidebar.Hovered();
		if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
			const auto d = GetMouseDelta();
			heading += d.x * 0.006f;
			pitch = std::clamp(pitch + d.y * 0.006f, -0.3f, 1.45f);
		}
		if (!overSidebar) distance = std::clamp(distance * (1.0f - GetMouseWheelMove() * 0.1f), 0.8f, 30.0f);
		const JPH::Vec3 forward(std::sin(heading), std::cos(heading), 0), right(std::cos(heading), -std::sin(heading), 0);
		// Arrow keys walk: Q, W and E are the three death shots.
		JPH::Vec3       moveVel = forward * static_cast<float>(down(KEY_UP) - down(KEY_DOWN)) +
		                    right * static_cast<float>(down(KEY_RIGHT) - down(KEY_LEFT));
		if (moveVel.Length() > 0.0f)
			moveVel = moveVel.Normalized() * (down(KEY_LEFT_SHIFT) ? Lab::g_config.puppet.runSpeed : Lab::g_config.puppet.walkSpeed);
		if (pressed(KEY_R)) {
			world.Reset();
			puppeting = puppet.Capture(world, world.StartYaw());
		}
		if (pressed(KEY_THREE)) puppeting = !puppeting;  // limp: the controller lets go; back on, it gets up from wherever it lies
		if (pressed(KEY_FOUR)) {  // soft walker that can fall
			puppet.weak = !puppet.weak;
			puppeting = true;
		}
		if (pressed(KEY_ONE) && puppeting) {  // knocked down: upper body shoved along the camera, gets up by itself
			for (int i = 0; i < world.PartCount(); ++i)
				if (puppet.IsTrunk(i) && i != 0) world.Bodies().AddLinearVelocity(world.Body(i), forward * g_sim.pushSpeed);
			puppet.KnockDown();
		}
		if (pressed(KEY_F)) {
			for (int i = 0; i < world.PartCount(); ++i)
				if (world.Part(i).name == "Chest") world.Bodies().AddLinearVelocity(world.Body(i), forward * g_sim.pushSpeed);
		}
		if (pressed(KEY_P)) paused = !paused;
		// Killing shot along the mouse ray at the body part under it: Q death walk, W death stagger, E death stumble; left
		// click rolls which one like the plugin (Death::Roll), and may roll none (a plain ragdoll fall).
		const bool rolled = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
		const bool shot = !overSidebar && !typing && (rolled || pressed(KEY_Q) || pressed(KEY_W) || pressed(KEY_E));
		if (shot) {
			const Ray       ray = GetScreenToWorldRayEx(GetMousePosition(), camera, viewW, viewH);
			const JPH::Vec3 dir(ray.direction.x, ray.direction.y, ray.direction.z);
			if (const int part = world.Pick(JPH::Vec3(ray.position.x, ray.position.y, ray.position.z), dir); part >= 0) {
				bool                             grip = true;
				const std::optional<Death::Kind> kind = rolled            ? Death::Roll(world, part, grip)
				                                        : pressed(KEY_Q) ? Death::Kind::kWalk
				                                        : pressed(KEY_W) ? Death::Kind::kStagger
				                                                              : Death::Kind::kStumble;
				if (kind) death.Start(world, part, dir, kShotScale, *kind, grip);
				else death.Stop();
				puppeting = false;
			}
		}
		if (puppeting) death.Stop();  // R, 1, 3, 4: the puppet is back in charge

		// Fixed 60 Hz steps; the controller runs before each one, like the game's PreStep listener.
		int stepsToRun = 0;
		if (!paused) {
			accumulator = std::min(accumulator + frame * g_sim.timeScale, 4 * kStep);
			for (; accumulator >= kStep; accumulator -= kStep) ++stepsToRun;
		} else if (pressed(KEY_N)) {
			stepsToRun = 1;
		}
		for (int i = 0; i < stepsToRun; ++i) {
			if (puppeting) puppet.Step(world, moveVel, kStep);
			death.Step(world, kStep);
			world.Step(kStep);
		}

		// Camera follows the pelvis.
		const JPH::Vec3 pelvis = JPH::Vec3(world.Bodies().GetPosition(world.Body(0)));
		focus = focus + (pelvis - focus) * std::min(1.0f, frame * 6.0f);
		camera.target = ToRl(focus);
		camera.position = ToRl(focus - JPH::Vec3(std::cos(pitch) * forward.GetX(), std::cos(pitch) * forward.GetY(), -std::sin(pitch)) * distance);

		BeginDrawing();
		ClearBackground({ 30, 32, 38, 255 });
		BeginMode3D(camera);
		{  // the 3D view's own viewport and aspect (BeginMode3D sets them for the whole window)
			const float scale = float(GetRenderWidth()) / float(std::max(1, GetScreenWidth()));
			rlViewport(0, 0, int(viewW * scale), int(viewH * scale));
			const double top = RL_CULL_DISTANCE_NEAR * std::tan(camera.fovy * 0.5 * DEG2RAD), side = top * viewW / viewH;
			rlMatrixMode(RL_PROJECTION);
			rlLoadIdentity();
			rlFrustum(-side, side, -top, top, RL_CULL_DISTANCE_NEAR, RL_CULL_DISTANCE_FAR);
			rlMatrixMode(RL_MODELVIEW);
		}
		DrawScene(world, puppet, death, puppeting, focus);
		EndMode3D();
		rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());

		if (death.Active())
			DrawText(TextFormat("%s %.1f s left   steps %u   x%.2f%s", Death::KindName(death.kind), death.Left(), death.steps, g_sim.timeScale,
						 paused ? "   PAUSED" : ""),
				12, 10, 20, RAYWHITE);
		else
		DrawText(TextFormat("%s %s   steps %u   x%.2f%s", puppeting ? (puppet.weak ? "PUPPET (weak)" : "PUPPET") : "LIMP", puppeting ? Puppet::RiseName(puppet.rise) : "", puppet.steps,
					 g_sim.timeScale, paused ? "   PAUSED" : ""),
			12, 10, 20, RAYWHITE);
		DrawText("LMB shoot (plugin's roll)   Q/W/E shoot at mouse (death walk / stagger / stumble)   arrows walk (Shift run)", 12, 36, 16, LIGHTGRAY);
		DrawText("1 knock down   3 puppet on/off   4 weak on/off   F push   R reset", 12, 56, 16, LIGHTGRAY);
		DrawText("P pause   N single step   RMB orbit   wheel zoom", 12, 76, 16, LIGHTGRAY);
		DrawText(settings.status.c_str(), 12, GetScreenHeight() - 46, 16, GRAY);
		DrawText(pluginSettings.status.c_str(), 12, GetScreenHeight() - 26, 16, GRAY);
		auto table = PluginTable(g_plugin);
		if (const auto action = sidebar.Draw(table, pluginSettings.path.filename().string().c_str()); action == Sidebar::Action::kSave) {
			std::string saveError;
			if (Lab::SaveSettings(pluginSettings.path, table, saveError)) {
				sidebar.dirty = false;
				fileValues = g_plugin;
				sidebar.message = "saved to " + pluginSettings.path.string();
			} else {
				sidebar.message = "save failed: " + saveError;
			}
			std::puts(sidebar.message.c_str());
		} else if (action == Sidebar::Action::kReset) {
			g_plugin = fileValues;
			sidebar.dirty = false;
			sidebar.message = "reset to the saved values";
		}
		DrawFPS(viewW - 90, 10);
		EndDrawing();
	}
	sidebar.Unload();
	CloseWindow();
	return 0;
}
