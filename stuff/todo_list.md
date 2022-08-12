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
[x] Fix ray tracer switch case in order for it to be able to see through doors.
[x] Write a ros viewhelper function to display the associations of only one pose + consider using dynamic reconfigure
[x] The ray tracer needs to be adjusted: it is okay to pass through single positive/negative cells, if there is no zero crossing, these
    cells should never be added to the associations though, only those belonging to an actual zero crossing. goddamn. in other words:
    i need to save these cells and add them later on, given the criteria, that a zero crossing is actually present.
    Doors for example are a huge problem right now, as rays can't pass through them
[x] Stop hardcoding tsdf values in code (600, 0, ...)
[x] or: find a different way to store the association information
[x] fixed enormous bug: when trying to find the bresenham vertices, we need to round towards the center of the localmap, not towards zero
[x] Use global Map parameters
[x] Create a model for the metadata of the global map ( which not yet exists)
[x] The ray tracer will always miss some cells, as the growth factor needs to be infinetly small the closer the ray gets to corners of tsdf cells
    This cannot be adressed the way the tracer is currently build, quite probably, this will lead to a lot of problems. Might be useful to change to Bresenham here
    memory intensive though and should probably be calculated before trying out. (roughly 3GB)
    DONE - this doesnt change a lot, since bresenham has similar issues.

Intersection Data:

[x] The intersection data should be written to a seperate group containing datsets inscribed by the chunk name
[x] {WIP:} Write a loop closure detection using min distance traveled 'd', max distance between known pose "d_max" and a visibility criteria using the ray tracer.
[x] {WIP:} Test the loop closure detection and is_visible method
[x] -o3 gcc flag for cmake? optimization -> TODO: eval optimization
## Interesting but not necessary atm ##

[ ] Outsource raytracers function to detect raytrace status changes to a function for that
[ ] PathExploration: Use tsdf cells instead of chunks (might be taking WAY to long, probably just another discretization level might be useful)
[ ] Hit percentage needs to be calculated differently (possibly), not complete local map, but just the visible chunks should be included. (of the sub path belonging to the loop closure)
[ ] RayTracer: data should not be reinitialized every time there is a tracing, there should be one default relative instanciation
[ ] adjust runtime using OPENMP
[ ] include timetracker from fastsense
[x] get a better understanding on weights (using the original doc thesis)
[ ] See, if there might be a quick fix for the path finder, which fixes issues with out of bounds vertices, same as bresenham! (relative real to world)
[ ] BUG: during the loop closure process, the map is enlarged by empty chunks, probably due to some indexing problem. If there is time, try to fix this.
[ ] Keep on working on writing the intersectionstatus to the hdf5
[ ] Write and read IntersectionStatus to / from HDF5

## TODO's ##

[ ] read paper on loop closure, write down most important points
[ ] Re-evaluate cleanup artifacts method from global map
[ ] work on code todos, especially for bresenham
[ ] when updating the cells later on, possibly favor more recent positions over old ones (sinus/cosinus function)

[ ] The association data should only be determined, when an actual loop is present in the map
[ ] Write the update process of the global map, when a loop was found
[ ] write a job, which removes the cells, which have not been covered during bresenham/raytracing


## Important TODO's for future me ##


## ASAP ##
[ ] Update local and global map with a feature, which allows writing the Intersection status to the hdf5 and reading it
[ ] Integration of GTSAM for Loop Closure optimization: https://gtsam.org/tutorials/intro.html


[ ] Implement the ray tracer as a singleton, so the rays don't need initialization every time. (insert this into masters thesis)
    -> There needs to be an additional array (e.g. init_rays)
    -> function to destroy current instance and make available for reinstanceiation
[ ] Implement tests to ensure writing to the globalmap works, write meta data (e.g. hit percentage, number hit vs total occupied cells)
    to hdf (/associations)

[ ] Bresenham: precalc of thee finish vertices not necessary, just use the direction vector and do an inbounds() check with the localmap
[ ] The current approach is (somewhat) errornous. We should look for one loop in the map, create associations in between these poses, update the map and look again
    This needs to be addressed and updated.

## TODAY ##

# Main target #

# What needs to be done? #



## Junkyard ##

[-] define the serialization of the path to json
[-] Update local map with a feature, which returns global index coordinates for a 3d point
[-] Add functionality for a Pose to add, substract etc. (will be needed) 
[?] create a way ( in the localmap ) to read, which part of the globalmap can be seen from a certain position
[?] create a method in the localmap, which checks the current intersection of a ray with the localmap
[-] Might also be an idea (as per Julian) to pre-calculate all the steps you need to take for a tracer in bounds of the local map. this might be pretty 
[-] write a ros node, which converts a globalmap h5 to a more usable version including metadata and intersection status.
[?] Associations: Different strat: one file, group for every chunk, inside group for every pose, containing the associated Poses.