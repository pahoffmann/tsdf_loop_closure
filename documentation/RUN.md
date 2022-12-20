# Running the Project

## Prerequisits:

* Install needed librarys:
* [README.md](../README.md)

## Ros-Nodes

This package contains multiple ROS-Nodes, most of them solely used to benchmark or test specific features of the project. Others are still usable, though deprecated, as part of previous research done on the topic of loop closure application in TSDF-based maps, resulting in SLAM approaches.

The nodes of interest, which are currently in use are:

### Data Listeners:
[Slam6dListener](../src/nodes/slam6d_listener.cpp)


### Data Publishers
[TransformCoordSys](../src/nodes/transform_coord_sys.cpp)
[PcdHDF5Publisher](../src/nodes/data_publishers/pcd_hdf5_publisher.cpp)


### Approach
The transform coord-sys publisher is used to read pointcloud and pose data from the Hannover1 Dataset (see: [Hannover1](http://kos.informatik.uni-osnabrueck.de/3Dscans/)). The PcdHDF5Publisher reads pointcloud data in form of PCDs and takes poses saved in an existing HDF5-File, which is the initial path estimation. This data can come from either be the result of the current Fastsense implementation ([FastSense](https://gitlab.informatik.uni-osnabrueck.de/FastSense/fastsense)) or the current warpsense implementation ([WarpSense](https://gitlab.informatik.uni-osnabrueck.de/FastSense/warpsense)). For both cases, data exists on the Glumanda-Server: glumanda.informatik.uni-osnabrueck.de.

The data of the publishers is then read by the SLAM6D-Listener, which is attached to the publishers using [Bondcpp](http://wiki.ros.org/bondcpp) (a ros package). The data is interpreted by the listener and an optimization of the inital path estimation using loop closures takes place. Furthermore, a TSDF-based map is generated and optimized, when a loop is identified.


### Launch files

To start these processes, multiple launch files have been written to simplify the launching.

The once interesting for execution are:

[PcdListener](../launch/pcd_hdf5_listener.launch)
[Slam6DListener](../launch/slam6d_listener.launch)

