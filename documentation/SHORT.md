# Short instructions:

No short way to success! Use this instead: [Run.md](RUN.md)

Kidding. If the prerequisits are matched ([Readme.md](../README.md)), so all libraries are installed and the packages are cloned into the ros workspace, build the workspace using:

###
    catkin_make -DCMAKE_BUILD_TYPE=Release

Afterwards, download the "physik_unten" or "vorplatz" dataset (.h5 and folder with .pcd's) from the glumanda server and place them at a dedicated location on your local machine. Location on glumanda:

###
    ~/pahoffmann/physik_unten

Open the general configuration file: [Configuration](../params/general_params.yaml) and replace the five parameters:

* /data_set/h5_file_name
  * location of the initial map and path estimate (.h5)
* /data_set/pcd_cloud_location
  * location of the belonging pointclouds (.pcd)
* /map/dir
  * folder, where the generated .h5 is saved
* /loop_closure/csv_save_path
  * path to save evaluation data and resulting clouds to

Now you are all set to start the algorithm:

###
    roslaunch loop_closure pcd_hdf5_listener.launch

To visualize the results, use the rviz software and the rviz config provided by this package.

[Rviz-Config](../slam6d.rviz)

If any errors occur, please consider checking the more detailed instructions:

[Run.md](RUN.md)