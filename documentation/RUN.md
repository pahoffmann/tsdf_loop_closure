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

When executing these files, no additional parameters need to be added:

###
    roslaunch loop_closure slam6d_listener.launch
    roslaunch loop_closure pcd_hdf5_listener.launch

Each command will start the respective publisher and listener and connect them using bondcpp. The current state of the algorithm can then be observed using rviz. To get a quick start, this package already provides a rviz configuration explicitly designed for this visualization:

[Rviz-Config](../slam6d.rviz)

Before launching, a few parametrization steps need to be executed first:

### Configuration

The main configuration is done Using [RosParam](http://wiki.ros.org/rosparam). Currently, there are two main configs providing the necessary parameters to run the code.

There is a generalized config specifically tailored to results of Fastsense or Warpsense and one used for the Hannover1 Dataset. This is necessary, because both types of initial estimations have a completely different foundation, with the Hannover1 Datset solely consisting of a Odometry estimation and the Results of Warpsense of Fastsense being generally way more accurate. This leaves a bigger margin for optimization in terms of the Hannover1 dataset or similar datasets.

The configs are:

[General](../params/general_params.yaml)
[Slam6D](../params/slam6d_listening.yaml)

Each of the configuration files share close to the same configurable fields, with most of them not needing to be adapted. Generally speaking, only an adaption of the dataset location is necessary to start the process.
These fields are to be found under the "data_set" section in the configs.

Further parameters that need to be changed:

* /map/dir
  * folder, where the generated .h5 is saved
* /loop_closure/ground_truth_filename
  * filename of the ground truth of the hannover1 dataset
* /loop_closure/csv_save_path
  * path to save evaluation data and resulting clouds to

### Datasets

The datasets itself are not delivered with this package, but to be found on the glumanda server of the university of osnabrueck. For those, who have got access to this server, i prepared multiple datasets to try out this algorithm:

* Datensatz des Physik Gebäudes der Uni Osnabrück
* Datensatz des Vorplatzes des Chemnitzer Opernhauses
* Hannover1 Datensatz der Uni Osnabrück

They are located on the glumanda server under:

###   
    ~/pahoffmann/hannover1
    ~/pahoffmann/vorplatz
    ~/pahoffmann/physik_unten

Copy those datasets to your local system and interchange the respective paths in the configs. After building the ros workspace, the algorithm can be started using the stated roslaunch commands. The output should be visible in the RVIZ program.

Please don't hesitate to ask questions, if an error occurs:
pahoffmann@uni-osnabrueck.de