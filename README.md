# Re:sim
Multi-agent traffic simulation software


## Requirement

The software is developed using Qt5.12 with MinGW64 compiler for Windows.
Some windows API should be replaced for running on Linux and Mac.

Re:sim uses freetype libraries for text rendering.
Please get following libraries appropriate for your platform.
  - freetype
  - libpng
  - zlib
  - bzip2
  
The library setting written in the Reisim.pro file should be modified according to your environment.
For Windows 10, you can find these libraries at Win10 folders.

## Usage

The shaders and true-type font should be placed in the same folder of Reisim.exe.


## Licence

[LGPL v3](https://github.com/Reisim/Reisim/LICENCE)
