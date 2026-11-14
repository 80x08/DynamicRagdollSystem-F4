#pragma once

// Reading and writing a Setting table as TOML (toml11). Header-only.

#include "Config.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <ostream>
#include <string>
#include <string_view>
#include <toml.hpp>

namespace Lab
{
	inline const toml::value* FindSetting(const toml::value& a_root, const char* a_section, const char* a_key)
	{
		if (!a_root.is_table() || !a_root.contains(a_section) || !a_root.at(a_section).is_table() || !a_root.at(a_section).contains(a_key))
			return nullptr;
		return &a_root.at(a_section).at(a_key);
	}

	// Missing keys, or keys of the wrong type, keep their current value; numbers are clamped to the table's range.
	inline void ApplySettings(const toml::value& a_root, const std::vector<Setting>& a_table)
	{
		for (const auto& s : a_table) {
			const auto* v = FindSetting(a_root, s.section, s.key);
			if (s.flag) {
				if (v && v->is_boolean()) *s.flag = v->as_boolean();
				continue;
			}
			if (!v || !(v->is_floating() || v->is_integer())) continue;
			const double parsed = v->is_floating() ? v->as_floating() : static_cast<double>(v->as_integer());
			if (std::isfinite(parsed)) *s.value = std::clamp(static_cast<float>(parsed), s.min, s.max);
		}
	}

	inline void WriteSettings(std::ostream& a_out, const std::vector<Setting>& a_table)
	{
		std::string_view section;
		for (const auto& s : a_table) {
			if (section != s.section) a_out << "\n[" << (section = s.section) << "]\n";
			a_out << std::format("# {} Range {}-{}.\n{} = {}\n", s.help, s.min, s.max, s.key, *s.value);
		}
	}

	// Writes a_table's values into the existing TOML file in place: each key's value line in its section gets the new value,
	// keeping everything else (comments, other keys, line endings). A key not in the file goes right under its section
	// header; a missing section is added at the end. Written to a temporary file first, so a failure leaves it untouched.
	inline bool SaveSettings(const std::filesystem::path& a_path, const std::vector<Setting>& a_table, std::string& a_error)
	{
		std::ifstream in{ a_path, std::ios::binary };
		if (!in) {
			a_error = "cannot read " + a_path.string();
			return false;
		}
		const std::string text{ std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>() };
		in.close();
		const std::string        eol = text.find("\r\n") != std::string::npos ? "\r\n" : "\n";
		std::vector<std::string> lines;
		for (std::size_t start = 0; start < text.size();) {
			std::size_t end = text.find('\n', start);
			if (end == std::string::npos) end = text.size();
			std::string line = text.substr(start, end - start);
			if (!line.empty() && line.back() == '\r') line.pop_back();
			lines.push_back(std::move(line));
			start = end + 1;
		}
		const auto trim = [](std::string_view a_s) {
			const auto b = a_s.find_first_not_of(" \t"), e = a_s.find_last_not_of(" \t");
			return b == std::string_view::npos ? std::string_view{} : a_s.substr(b, e - b + 1);
		};
		const auto number = [](const Setting& a_s) {  // a float, so TOML reads it as one (80 -> 80.0); a flag as true/false
			if (a_s.flag) return std::string(*a_s.flag ? "true" : "false");
			std::string s = std::format("{}", *a_s.value);
			if (s.find_first_of(".e") == std::string::npos) s += ".0";
			return s;
		};
		for (const auto& s : a_table) {
			std::string_view section;
			int              header = -1;
			bool             written = false;
			for (std::size_t i = 0; i < lines.size() && !written; ++i) {
				const auto line = trim(lines[i]);
				if (line.starts_with('[') && !line.starts_with("[[")) {
					section = trim(line.substr(1, line.find(']') - 1));
					if (section == s.section) header = static_cast<int>(i);
					continue;
				}
				if (section != s.section || !line.starts_with(s.key) || !trim(line.substr(std::string_view(s.key).size())).starts_with('=')) continue;
				const auto eq = lines[i].find('='), comment = lines[i].find('#', eq);
				lines[i] = lines[i].substr(0, eq + 1) + " " + number(s) + (comment != std::string::npos ? "  " + lines[i].substr(comment) : "");
				written = true;
			}
			if (written) continue;
			const std::string entry = std::string(s.key) + " = " + number(s);
			if (header >= 0) lines.insert(lines.begin() + header + 1, entry);
			else lines.insert(lines.end(), { "", std::string("[") + s.section + "]", entry });
		}
		std::string out;
		for (std::size_t i = 0; i < lines.size(); ++i) out += lines[i] + (i + 1 < lines.size() || text.ends_with('\n') ? eol : "");
		auto temp = a_path;
		temp += ".tmp";
		{
			std::ofstream file{ temp, std::ios::binary | std::ios::trunc };
			if (!(file << out) || !file.flush()) {
				a_error = "cannot write " + temp.string();
				return false;
			}
		}
		std::error_code ec;
		std::filesystem::rename(temp, a_path, ec);
		if (ec) {
			a_error = "cannot replace " + a_path.string() + ": " + ec.message();
			std::filesystem::remove(temp, ec);
			return false;
		}
		return true;
	}
}
