# loop_closure


# REQUIREMENTS (Tested on Ubuntu 20.04)

## ROS (Noetic) ##

[Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)

## GTSAM: ##


Used to optimize the path when a loop closure has been detected.

Github: https://github.com/borglab/gtsam

### Installation:

    git clone https://github.com/borglab/gtsam
    mkdir build
    cd build
    cmake ..
    sudo make install

## PCL ##

Usually delivered with ROS

## Eigen ##

Provides 3d objects like matrices and vectors used for 3d maths

### Installation:

    * sudo apt-get install libeigen3-dev

# Troubleshooting #

## GTSAM ##

Libmetis:

When occuring an error like "error while loading shared libraries: libmetis-gtsam.so: cannot open shared object file: No such file or directory",
a solution would be to add this line to your .bashrc:

export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH

This is a bug in Ubuntu 18.04 and beyond, as described in https://github.com/borglab/gtsam/issues/380

# HightFive

* Used to save and read data from a h5 file
* delivered with the project
* when cloning, submodules need to be initialized and updated:

####
    git submodule init
    git submodule update



# Get it running

[Run.md](documentation/RUN.md)