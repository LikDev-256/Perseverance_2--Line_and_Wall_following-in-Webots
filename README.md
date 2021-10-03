# Perseverance_2
# Line_and_Wall_following-in-Webots
#### This is a project on a Simulated Line following and Wall following robot in [Webots Simulation Software](https://github.com/cyberbotics/webots)

<img src='https://raw.githubusercontent.com/LikDev-256/Perseverance_2--Line_and_Wall_following-in-Webots/main/Videos/giphy.gif' width=650/>

## Getting Started

## Main Line Follower Robot

The Robot Body is made using SOLIDWORKS and from the use of a [**addon**](http://wiki.ros.org/sw_urdf_exporter). The CAD models are convertered into URDF files and Using the webots [URDF2PROTO converter](https://github.com/cyberbotics/urdf2webots) script.

The CAD models are imported into webots using the above methods. The urdf files are provided in here "$HOME/Perseverance_2_CAD_files"
The STL files can be found in "$HOME/Perseverance_2_CAD_files/meshes".
The Solidworks .SLASM file has unfortunately corrupted.

Distance sensors used in this Robot are SHARP 4 - 30cm range sensors
The equation in the official SHARP Library is used to calculate the RAW distance into CM.

Two Ground Sensor arrays are used to take Two point measurement get the exact position of the robot accurately

## Secondary Reciever Robot

This robot only has a **reciever** and **two leds** which waits for the **colour informations** which would be given by the **main line-follower emitter**

## The task Details are explained in the [PDF](https://github.com/LikDev-256/Perseverance_2--Line_and_Wall_following-in-Webots/blob/main/Final%20task%20description.pdf)

## Known Issues

### (SOLVED)
1. **C++ Libraries may be a issue** `#include <boost/algorithm/clamp.hpp>` .
   In a linux system a special library has to be installed to use `sudo apt-get install lib32z1 libncurses5 lib32stdc++6`.
   
   In Windows you can install the appropriate libraries through
   
   >goto " https://www.boost.org/users/download/ "
   >goto "Prebuilt windows binaries"
   >click on the version you want
   >dowload the last .exe version
   >install and it's all good

  ## Help or Contribution Need
  _**The Wall following is not perfect and has some rough edges**_
