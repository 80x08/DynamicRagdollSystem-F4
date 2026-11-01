# DynamicRagdollSystem

F4SE plugin for Fallout 4 1.10.163 (OG), built with [CommonLibF4](https://github.com/Dear-Modding-FO4/commonlibf4)
and xmake.

## Requirements

- Windows
- Visual Studio 2026 with the **Desktop development with C++** workload (MSVC)
- [xmake](https://xmake.io), installed to the default folder (`%LOCALAPPDATA%\Programs\xmake`)
- Git

## Get the source

CommonLibF4 is a submodule with its own submodules, so clone recursively:

```
git clone --recursive https://github.com/80x08/DynamicRagdollSystem-F4 DynamicRagdollSystem
```

Already cloned without it:

```
git submodule update --init --recursive
```

## Build

First build (configures xmake for `releasedbg` with MSVC):

```
build.bat config
```

After that:

```
build.bat
```

Output: `build\windows\x64\releasedbg\DynamicRagdollSystem.dll` (and `.pdb`).

`build.bat` also copies the DLL and PDB to `D:\Games\Fallout 4\Data\F4SE\Plugins\`. Change the two `copy` lines
at the end of `build.bat` to your own game folder, or remove them.

### Notes

- `build.bat` runs xmake inside the Visual Studio `vcvars64` environment. If clang is on your PATH, it otherwise
  takes over the dependency builds and they fail. If Visual Studio is installed somewhere else, fix the
  `vcvars64.bat` path at the top of `build.bat`.
- Building by hand works too, from a *x64 Native Tools Command Prompt for VS*:
  ```
  xmake f -m releasedbg --toolchain=msvc -y
  xmake build -y
  ```
- `-m debug` does not build (one of the dependency packages fails in debug mode); use `releasedbg`.
- The version is set in two places: `xmake.lua` (`set_version`) and `src/version.h`.

## License

MIT, see `LICENSE`. The plugin is built with [CommonLibF4](https://github.com/Dear-Modding-FO4/commonlibf4), which
is GPL-3.0; a compiled DLL contains it and is subject to its terms.
