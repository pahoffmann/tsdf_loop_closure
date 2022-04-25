## TODO's and DONE's ##

# DONE's #

[x] Extract Ray_Trace testing functionality to a seperate node, in test folder, actual loop closure needs to be clean of any testing code
[x] use globalmap and localmap from fastsense, adapt everything needed  !!DONE!!
[x] check the functionality of the globalmap to create a map from file  !!DONE SELFMADE!!
[x] start using the maps efficently !! WIP !!
[x] find a way to display intersections in a nice manner (e.g. some weird weight or a new variable)
[x] fix bugs in intersections
[x] possibility to do ray tracing from different positions
[x] define the data mapping problem -> which "data" comes from which scan? and: where do we actually put it kekw?
[x] define a way to deserialize the path (hdf5/json) possibly -> json
[x] Use boost program options for actual loop closure instead of dynamic reconfigure (maybe)
[x] In Loop closure node, the indexing of the local map is not correct now, as the starting pose is no longer (0,0,0), fix this
[x] Check association file and directory creation, as it does not work as intended.

# TODO's #

[ ] read paper on loop closure, write down most important points
[ ] define the serialization of the path to json
[ ] Update localmap with a function, that converts a Vector3f in real word coordinates to map coordinates (Vector3i) and vice versa
[ ] Outsource raytracers function to detect raytrace status changes to a function for that

# Important TODO's for future me #

[ ] get a better understanding on weights (using the original doc thesis)
[ ] Update local and global map with a feature, which allows writing the Intersection status to the hdf5 and reading it
[ ] Update local map with a feature, which returns global index coordinates for a 3d point
[ ] Cleanup job for global map, removing artifacts, single floating tsdf cells etc.

# ASAP #

# Junkyard #

[-] Add functionality for a Pose to add, substract etc. (will be needed) 
[?] create a way ( in the localmap ) to read, which part of the globalmap can be seen from a certain position
[?] create a method in the localmap, which checks the current intersection of a ray with the localmap