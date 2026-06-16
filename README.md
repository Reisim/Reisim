# Re:sim
Multi-agent traffic simulation software

<img width="980" alt="resim_image1" src="https://user-images.githubusercontent.com/60654261/81361024-3a34c180-9118-11ea-98bd-16d56e49d848.png">

## Platform Support

| Platform | Status |
|----------|--------|
| Windows 10 (Qt5.12 + MinGW64) | ✅ Official |
| macOS (Qt5.15 + Homebrew, Intel x86_64) | ✅ Community port ([PR#1](https://github.com/Reisim/Reisim/pull/1)) |

## Requirements

- freetype
- libpng
- zlib
- bzip2

**Windows**: pre-built libraries are in the `Win10/` folder.

**macOS** (Homebrew):
```bash
brew install qt@5 freetype libpng zlib bzip2
```

## Build

### Windows
Use Qt5.12 with MinGW64. Library paths are pre-configured in `Reisim.pro`.

### macOS
```bash
mkdir build-Reisim && cd build-Reisim
/usr/local/opt/qt@5/bin/qmake CONFIG+=sdk_no_version_check ../Reisim/src/Reisim.pro
make -j$(sysctl -n hw.logicalcpu)
cp ../Reisim/shaders_fonts/* Reisim.app/Contents/MacOS/
/usr/local/opt/qt@5/bin/macdeployqt Reisim.app
open Reisim.app   # use 'open', not direct binary execution
```

> **macOS note**: Always launch via `open Reisim.app` or Finder/Launchpad.
> Running the binary directly from a terminal that has Homebrew Qt on `DYLD_LIBRARY_PATH`
> causes a Qt version conflict with the bundled frameworks.

## Usage

Before running a simulation, prepare road network data using [SEdit](https://github.com/Reisim/SEdit).

1. Open Re:sim
2. Click **Open** (or ⌘O / Ctrl+O) → select a `*.rc.txt` configuration file
3. Click **Play** to start the simulation

<img width="512" alt="resim-image2" src="https://user-images.githubusercontent.com/60654261/81362215-42dac700-911b-11ea-84c8-106bc923f891.png">

<img width="512" alt="resim-image3" src="https://user-images.githubusercontent.com/60654261/81362231-4d955c00-911b-11ea-9184-892efcbb9b7d.png">

## Sample Data

The `Resim-Simulation-Sample/` folder contains a ready-to-use city map:

| File | Description |
|------|-------------|
| `CityMapData.se.txt` | Road network (open in SEdit) |
| `simdata_citymap.rc.txt` | Config file — open this in Re:sim to run |
| `simdata_citymap.rr.txt` | Road runtime data |
| `simdata_citymap.rs.txt` | Scenario |
| `simdata_citymap.ts.txt` | Traffic signals |
| `baseMapSS/` | Satellite map background tiles |

```bash
# Run the sample
open Reisim.app
# Load: Resim-Simulation-Sample/simdata_citymap.rc.txt
# Then press Play
```

## Binary (Windows)

Complete Windows binaries (EXE + DLLs): [bin_win10/Resim.zip](https://github.com/Reisim/Reisim/tree/master/bin_win10)

## Manuals

- [User Manual (PDF)](https://github.com/Reisim/Reisim/blob/master/Resim-Manual.pdf)
- [Quick Start (PDF)](https://github.com/Reisim/Reisim/blob/master/quick_start.pdf)
- SEdit Manuals: [Part 1](https://github.com/Reisim/SEdit/tree/master/SEdit-Manual-Part1-Basic_Operation.pdf) · [Part 2](https://github.com/Reisim/SEdit/tree/master/SEdit-Manual-Part2-Scenario_Data_Setting.pdf)

## Licence

[LGPL v3](https://github.com/Reisim/Reisim/blob/master/LICENSE)
