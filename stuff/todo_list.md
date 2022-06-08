## TODO's and DONE's ##

## DONE's ##

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
[x] Update localmap with a function, that converts a Vector3f in real word coordinates to map coordinates (Vector3i) and vice versa
[x] Cleanup job for global map, removing artifacts, single floating tsdf cells etc.
[x] PathExploration: No more Manhatten neighboring
[x] PathExploration: weighting trough raytrace (specific percentage of hit vs non hit)
[x] where is the intersection status saved??? Well well, it is not written back, so thats why it's never "saved"
[x] HDF5 structure overthinking.
[x] Outsource HDF Constants (datset names etc.) to a constants file for hdf5
[x] Evaluate: should the association data be written to hdf5? would make sense right? Yes.
[x] The HDF Structure needs some rework... don't create empty datasets etc.
[x] The methods "write_path" and several calls to "write_path_node" should lead to the same results

## TODO's ##

[ ] read paper on loop closure, write down most important points
[ ] Hit percentage needs to be calculated differently (possibly), not complete local map, but just the visible chunks should be included. (of the sub path belonging to the loop closure)
[ ] Outsource raytracers function to detect raytrace status changes to a function for that
[ ] Create a model for the metadata of the global map ( which not yet exists)
[ ] write a ros node, which converts a globalmap h5 to a more usable version including metadata and intersection status.
[ ] Re-evaluate cleanup artifacts method from global map

## Important TODO's for future me ##

[ ] Stop hardcoding tsdf values in code (600, 0, ...)
[ ] BUG: during the loop closure process, the map is enlarged by empty chunks, probably due to some indexing problem. If there is time, try to fix this.
[ ] get a better understanding on weights (using the original doc thesis)

## ASAP ##
[ ] Update local and global map with a feature, which allows writing the Intersection status to the hdf5 and reading it
[ ] or: find a different way to store the association information
[ ] PathExploration: Use tsdf cells instead of chunks (might be taking WAY to long, probably just another discretization level might be useful)
[ ] Associations: Different strat: one file, group for every chunk, inside group for every pose, containing the associated Poses.
[ ] Integration of GTSAM for Loop Closure optimization: https://gtsam.org/tutorials/intro.html
[ ] Write a loop closure detection using min distance traveled 'd', max distance between known pose "d_max" and a visibility criteria using the ray tracer.

## TODAY ##

# Main target #
[ ] Keep on working on writing the intersectionstatus to the hdf5

# What needs to be done? #

[ ] Write and read IntersectionStatus to / from HDF5

[ ] The intersection data should be written to a seperate group containing datsets inscribed by the chunk name

## Junkyard ##

[-] define the serialization of the path to json
[-] Update local map with a feature, which returns global index coordinates for a 3d point
[-] Add functionality for a Pose to add, substract etc. (will be needed) 
[?] create a way ( in the localmap ) to read, which part of the globalmap can be seen from a certain position
[?] create a method in the localmap, which checks the current intersection of a ray with the localmap