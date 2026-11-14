#pragma once

// The tuning sidebar, a panel of its own on the right of the window (the 3D view gets the rest): a slider per setting in a
// Lab::Setting table, grouped by section, with Reset and Save buttons. Drag a slider to change a value (Shift held: 10x
// finer) or click its value and type one (Enter sets it, Escape cancels); click a true/false box to flip it; the wheel
// scrolls the list. Text in Segoe UI (Windows' UI font), or raylib's own font if it is missing.

#include "ConfigIO.h"

#include <raylib.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

struct Sidebar
{
	static constexpr int kWidth = 400, kRow = 22, kTop = 64;
	enum class Action { kNone, kSave, kReset };

	float       scroll{};
	int         dragging{ -1 };
	bool        dirty{};    // changed since loaded or saved
	std::string message;    // last save / reset result

	// After InitWindow. Fonts are rasterised at the sizes drawn, so they stay sharp.
	void Load()
	{
		const auto load = [](const char* a_file, int a_size) {
			const std::string path = std::string("C:/Windows/Fonts/") + a_file;
			return FileExists(path.c_str()) ? LoadFontEx(path.c_str(), a_size, nullptr, 0) : GetFontDefault();
		};
		m_text = load("segoeui.ttf", kTextSize);
		m_bold = load("segoeuib.ttf", kTitleSize);
	}
	void Unload()
	{
		for (Font* f : { &m_text, &m_bold })
			if (f->texture.id != GetFontDefault().texture.id) UnloadFont(*f);
	}

	Rectangle Area() const { return { float(GetScreenWidth() - kWidth), 0.0f, float(kWidth), float(GetScreenHeight()) }; }
	bool      Hovered() const { return CheckCollisionPointRec(GetMousePosition(), Area()) || dragging >= 0; }
	bool      Typing() const { return m_editing >= 0; }  // a value is being typed: the keys are the sidebar's

	Action Draw(std::vector<Lab::Setting>& a_table, const char* a_title)
	{
		const Rectangle area = Area();
		const Vector2   mouse = GetMousePosition();
		DrawRectangleRec(area, { 24, 26, 31, 255 });
		DrawLine(int(area.x), 0, int(area.x), int(area.height), { 60, 64, 72, 255 });
		Text(m_bold, TextFormat("%s%s", a_title, dirty ? " *" : ""), area.x + 10, 10, kTitleSize, RAYWHITE);
		const auto button = [&](Rectangle a_r, const char* a_label, Color a_color) {
			const bool over = CheckCollisionPointRec(mouse, a_r);
			DrawRectangleRec(a_r, over ? ColorBrightness(a_color, 0.2f) : a_color);
			const Vector2 size = MeasureTextEx(m_text, a_label, kTextSize, 0);
			Text(m_text, a_label, a_r.x + (a_r.width - size.x) / 2, a_r.y + (a_r.height - size.y) / 2, kTextSize, RAYWHITE);
			return over && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
		};
		Action action = Action::kNone;
		if (button({ area.x + area.width - 150, 8, 64, 24 }, "Reset", { 100, 70, 50, 255 })) action = Action::kReset;
		if (button({ area.x + area.width - 78, 8, 64, 24 }, "Save", { 60, 100, 60, 255 })) action = Action::kSave;
		Text(m_text, message.c_str(), area.x + 10, 38, kTextSize, GRAY);

		// Sets a value: clamped to its range, and a Min/Max pair can't cross (the table lists Min right before Max).
		const auto set = [&](int a_i, float a_v) {
			auto&                  s = a_table[a_i];
			const std::string_view key = s.key;
			a_v = std::clamp(a_v, s.min, s.max);
			if (key.ends_with("Min") && a_i + 1 < int(a_table.size())) a_v = std::min(a_v, *a_table[a_i + 1].value);
			if (key.ends_with("Max") && a_i > 0) a_v = std::max(a_v, *a_table[a_i - 1].value);
			if (a_v != *s.value) *s.value = a_v, dirty = true;
		};
		// Typing a value (clicked on it): digits, '.', '-'; Enter or a click elsewhere sets it, Escape cancels. Escape
		// doesn't close the window meanwhile.
		if (m_editing >= 0) {
			for (int c = GetCharPressed(); c; c = GetCharPressed())
				if (((c >= '0' && c <= '9') || c == '.' || c == '-') && m_edit.size() < 12) m_edit += char(c);
			if ((IsKeyPressed(KEY_BACKSPACE) || IsKeyPressedRepeat(KEY_BACKSPACE)) && !m_edit.empty()) m_edit.pop_back();
			const bool commit = IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER) || IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
			if (commit && m_editing < int(a_table.size()) && a_table[m_editing].value) {
				char*       end = nullptr;
				const float v = std::strtof(m_edit.c_str(), &end);
				if (end != m_edit.c_str() && *end == 0 && std::isfinite(v)) set(m_editing, v);
			}
			if (commit || IsKeyPressed(KEY_ESCAPE)) {
				m_editing = -1;
				SetExitKey(KEY_ESCAPE);
			}
		}

		// Rows: a section header whenever the section changes, then one row per setting.
		const float list = area.height - kTop;
		float       content = 0.0f;
		const char* section = "";
		for (const auto& s : a_table) {
			if (std::string_view(section) != s.section) content += kRow, section = s.section;
			content += kRow;
		}
		if (CheckCollisionPointRec(mouse, area)) scroll -= GetMouseWheelMove() * kRow * 3;
		scroll = std::clamp(scroll, 0.0f, std::max(0.0f, content - list));

		BeginScissorMode(int(area.x), kTop, int(area.width), int(list));
		float y = kTop - scroll;
		section = "";
		for (int i = 0; i < int(a_table.size()); ++i) {
			auto& s = a_table[i];
			if (std::string_view(section) != s.section) {
				section = s.section;
				Text(m_bold, TextFormat("[%s]", section), area.x + 10, y + 3, kTitleSize, ORANGE);
				y += kRow;
			}
			const Rectangle bar{ area.x + 230, y + 8, 100, 7 };
			const bool      visible = y + kRow > kTop && y < area.height;
			const bool      press = visible && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && mouse.y >= kTop;
			if (s.flag) {  // true/false: a box, clicked to toggle
				const Rectangle box{ bar.x, y + 5, 12, 12 };
				if (press && CheckCollisionPointRec(mouse, { box.x - 4, y, 60, float(kRow) })) *s.flag = !*s.flag, dirty = true;
				if (visible) {
					Text(m_text, s.key, area.x + 10, y + 3, kTextSize, LIGHTGRAY);
					DrawRectangleLinesEx(box, 1, LIGHTGRAY);
					if (*s.flag) DrawRectangle(int(box.x) + 3, int(box.y) + 3, 6, 6, { 110, 170, 230, 255 });
					Text(m_text, *s.flag ? "true" : "false", bar.x + bar.width + 10, y + 3, kTextSize, RAYWHITE);
				}
				y += kRow;
				continue;
			}
			const Rectangle field{ bar.x + bar.width + 6, y + 2, 56, float(kRow - 4) };  // the value: click to type one
			if (press && CheckCollisionPointRec(mouse, { bar.x - 4, y, bar.width + 8, float(kRow) })) dragging = i;
			if (press && CheckCollisionPointRec(mouse, field)) {
				m_editing = i;
				m_edit = TextFormat("%g", *s.value);
				SetExitKey(KEY_NULL);
			}
			if (dragging == i && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
				const float range = s.max - s.min;
				const float v = IsKeyDown(KEY_LEFT_SHIFT) ? *s.value + GetMouseDelta().x / bar.width * range * 0.1f
				                                          : s.min + std::clamp((mouse.x - bar.x) / bar.width, 0.0f, 1.0f) * range;
				set(i, std::round(v * 1000.0f) / 1000.0f);
			}
			if (visible) {
				const float t = s.max > s.min ? (*s.value - s.min) / (s.max - s.min) : 0.0f;
				Text(m_text, s.key, area.x + 10, y + 3, kTextSize, dragging == i || m_editing == i ? YELLOW : LIGHTGRAY);
				DrawRectangleRec(bar, { 60, 64, 72, 255 });
				DrawRectangleRec({ bar.x, bar.y, bar.width * std::clamp(t, 0.0f, 1.0f), bar.height }, { 110, 170, 230, 255 });
				if (m_editing == i) {
					DrawRectangleRec(field, { 40, 44, 52, 255 });
					DrawRectangleLinesEx(field, 1, YELLOW);
					const bool caret = std::fmod(GetTime(), 1.0) < 0.5;
					Text(m_text, TextFormat("%s%s", m_edit.c_str(), caret ? "|" : ""), field.x + 4, y + 3, kTextSize, RAYWHITE);
				} else {
					if (CheckCollisionPointRec(mouse, field)) DrawRectangleLinesEx(field, 1, { 90, 96, 108, 255 });
					Text(m_text, TextFormat("%.4g", *s.value), field.x + 4, y + 3, kTextSize, RAYWHITE);
				}
			}
			y += kRow;
		}
		EndScissorMode();
		if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) dragging = -1;
		return action;
	}

private:
	static constexpr int kTextSize = 15, kTitleSize = 16;
	Font                 m_text{}, m_bold{};
	int                  m_editing{ -1 };  // row whose value is being typed
	std::string          m_edit;           // ... what has been typed

	static void Text(const Font& a_font, const char* a_text, float a_x, float a_y, int a_size, Color a_color)
	{
		DrawTextEx(a_font, a_text, { std::round(a_x), std::round(a_y) }, float(a_size), a_font.texture.id == GetFontDefault().texture.id ? 1.0f : 0.0f, a_color);
	}
};
