#pragma once

// Tuning values of the simulator. One table drives reading, clamping and writing
// RagdollLab.toml.

#include <vector>

namespace Lab
{
	struct Config
	{
		bool  hotReload = true;  // re-read the file whenever it is saved
		float ragdollForce = 4.0f;
		struct
		{
			float push = 4.0f, duration = 1.5f, softness = 0.1f, recoverTime = 0.8f, blendOut = 0.3f, catchUp = 7.0f;
		} stumble;
		struct
		{
			float walkSpeed = 1.4f, runSpeed = 3.0f, turnRate = 4.0f, leash = 0.35f, crouch = 0.9f;
			float kp = 100.0f, kd = 18.0f, maxAccel = 60.0f, spinKp = 60.0f, spinKd = 14.0f, maxSpin = 60.0f;
			float stepTime = 0.32f, stepLift = 0.15f, stepTrigger = 0.12f, stepLead = 0.5f, swingKp = 30.0f, swingAccel = 150.0f,
			      stanceHold = 60.0f;
			float weakSupport = 0.25f, giveUpAngle = 35.0f;
		} puppet;
	};
	inline Config g_config;

	struct Setting
	{
		const char* section;
		const char* key;
		float*      value;  // null for a true/false setting (flag)
		float       min, max;
		const char* help;
		bool*       flag = nullptr;
	};

	inline std::vector<Setting> ConfigTable(Config& c)
	{
		auto& s = c.stumble;
		auto& p = c.puppet;
		return {
			{ "Ragdoll", "fForce", &c.ragdollForce, 0, 50, "Key 1. Knockdown push, engine explosion magnitude (0 = just fall, 10 = launch). The NPC then gets up with physics and blends back into its animation." },
			{ "Stumble", "fPush", &s.push, 0, 50, "Key 2. Push on the upper body, m/s." },
			{ "Stumble", "fDuration", &s.duration, 0.2f, 10, "Seconds the body stays physical." },
			{ "Stumble", "fSoftness", &s.softness, 0, 1, "Pull toward the animation right after the push (fraction of the engine's own)." },
			{ "Stumble", "fRecoverTime", &s.recoverTime, 0.05f, 10, "Seconds to firm back up to full pull." },
			{ "Stumble", "fBlendOut", &s.blendOut, 0, 2, "Seconds at the end to fade back to pure animation." },
			{ "Stumble", "fCatchUp", &s.catchUp, 0.1f, 50, "m/s the ragdoll may close the gap to the animation." },
			{ "Puppet", "fWalkSpeed", &p.walkSpeed, 0, 10, "Key 3. m/s with WASD." },
			{ "Puppet", "fRunSpeed", &p.runSpeed, 0, 15, "m/s with WASD + Shift." },
			{ "Puppet", "fTurnRate", &p.turnRate, 0, 30, "rad/s the trunk turns toward the walking direction." },
			{ "Puppet", "fLeash", &p.leash, 0.05f, 3, "m the walk target may run ahead of the real pelvis." },
			{ "Puppet", "fCrouch", &p.crouch, 0.3f, 1.2f, "Lowest the pelvis goes while walking, fraction of standing height. It rides higher over a planted leg; lower = longer strides, more bent knees." },
			{ "Puppet", "fKp", &p.kp, 0, 500, "Trunk position spring, 1/s^2. Higher = stiffer strings, too high jitters." },
			{ "Puppet", "fKd", &p.kd, 0, 100, "Trunk position damping, 1/s. Higher = less bounce." },
			{ "Puppet", "fMaxAccel", &p.maxAccel, 0, 500, "m/s^2 cap per trunk body. Below ~15 it cannot hold its weight." },
			{ "Puppet", "fSpinKp", &p.spinKp, 0, 500, "Trunk orientation spring, 1/s^2." },
			{ "Puppet", "fSpinKd", &p.spinKd, 0, 100, "Trunk orientation damping, 1/s." },
			{ "Puppet", "fMaxSpin", &p.maxSpin, 0, 500, "rad/s^2 cap per trunk body." },
			{ "Puppet", "fStepTime", &p.stepTime, 0.05f, 2, "Seconds per step." },
			{ "Puppet", "fStepLift", &p.stepLift, 0, 1, "m the foot rises mid-step." },
			{ "Puppet", "fStepTrigger", &p.stepTrigger, 0.01f, 1, "m a foot may be off its spot before it steps." },
			{ "Puppet", "fStepLead", &p.stepLead, 0, 3, "Feet land this many step times ahead of the moving body." },
			{ "Puppet", "fSwingKp", &p.swingKp, 0, 200, "Swing leg spring toward its pose along the step, 1/s." },
			{ "Puppet", "fSwingAccel", &p.swingAccel, 0, 1000, "m/s^2 cap for the swing foot." },
			{ "Puppet", "fStanceHold", &p.stanceHold, 0, 200, "Planted foot and calf damping, 1/s. Keeps the standing leg from being dragged and kicking up." },
			{ "Puppet", "fWeakSupport", &p.weakSupport, 0.05f, 1, "Key 4. Trunk springs as a fraction of key 3's: lower = wobblier, easier to knock over." },
			{ "Puppet", "fGiveUpAngle", &p.giveUpAngle, 5, 90, "Key 4. Degrees the trunk may tip before it gives up, falls and gets up again." },
		};
	}
}
