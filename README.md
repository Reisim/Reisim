# Re:sim
Multi-agent traffic simulation software

<img width="980" alt="resim_image1" src="https://user-images.githubusercontent.com/60654261/81361024-3a34c180-9118-11ea-98bd-16d56e49d848.png">


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

Before running the simulation, data should be prepared using SEdit.

See [quick_start.pdf](https://github.com/Reisim/Reisim/blob/master/quick_start.pdf) for more detail.

## Binary for windows and Sample Data

You can get the binary files of Re:sim and S-Edit by downloading zip files in bin_win10 folder.
Unzip the Resim.zip and SEdit.zip and run Reisim.exe and SEdit.exe in the resultant folders.

The sample data is also provided, download the files in Resim-Simulation-Sample folder.
Open CityMapData.se.txt with SEdit.exe to see the simulation data.
To run the simulation, open simdata_citymap.rc.txt and press start button. 

<img width="512" alt="resim-image2" src="https://user-images.githubusercontent.com/60654261/81362215-42dac700-911b-11ea-84c8-106bc923f891.png">

<img width="815" alt="resim-image3" src="https://user-images.githubusercontent.com/60654261/81362231-4d955c00-911b-11ea-9184-892efcbb9b7d.png">


## Licence

[LGPL v3](https://github.com/Reisim/Reisim/blob/master/LICENSE)
