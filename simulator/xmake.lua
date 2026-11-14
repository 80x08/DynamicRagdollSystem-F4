-- RagdollLabSim: standalone Jolt Physics stand-in for Fallout 4's Havok ragdoll, for fast controller tuning.
-- Separate project from the plugin (its own xmake project; build it from this folder).
set_project("RagdollLabSim")
set_version("0.1.0")
set_languages("c++20")
set_warnings("allextra")

add_rules("mode.debug", "mode.releasedbg")

add_requires("joltphysics", "raylib", "nlohmann_json", "toml11 v4.4.0")

target("RagdollLabSim")
    set_kind("binary")
    add_files("src/*.cpp")
    add_headerfiles("src/*.h")
    add_includedirs("src")
    add_packages("joltphysics", "raylib", "nlohmann_json", "toml11")
    add_syslinks("user32", "gdi32", "winmm", "shell32")
    set_rundir("$(projectdir)")
