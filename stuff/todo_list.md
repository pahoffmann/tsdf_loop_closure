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
[x] Map update: i need to create a copy of the localmap in order to ensure no cell values are overwritten by accident
[x] Map update: there is currently an error in the indexing, because of which the cell seperations may contain weirrd cells. why is there a cell (0,0,0) in there?
[x] Fix out of bounds in map update
    -> There needs to be some form of clustering for the new and old cells to ensure, that we dont shift very often
[x] Rotate path: needs to consider the rotation of the point the path is supposed to be rotated around (or maybe not?)
    Does not, but we need to use radiants of course, not degrees
[x] add a test for the path translationm -> works
[x] check the rounding, when calculating the new cell poses. where do we need to round here? relative?
    -> nope, basic rounding, wrong thought process
[x] Testing: rotate the path around its center and check, what happens to the map -> map update definetly not working atm
[x] work on code todos, especially for bresenham
[x] Write the update process of the global map, when a loop was found
    -> This has been done, though the process doesnt really work the way i thought it would. basically need to rethink the whole thing
[x] The current approach is (somewhat) errornous. We should look for one loop in the map, create associations in between these poses, update the map and look again
    This needs to be addressed and updated.
[x] Problems with the map update: 
[x] A. After the update, there are only positive valued cells left...
[x] B. there is far less updates, than the code says... look into that
    All the problems were solved, when the deserialization was fixed... :)
[x] The association data should only be determined, when an actual loop is present in the map
[x] Check diff between association number in map update vs. number of cells displayed in rviz (intersection markers) - they are off...
[x] Write a test case, which checks the local map functionality... check if resetting all cells with value actually resets them. YUP
[x] Only generate associations in bounds of the loop closure indices. can do
[x] Implement tests to ensure writing to the globalmap works, write meta data (e.g. hit percentage, number hit vs total occupied cells)
    to hdf (/associations)
[x] implement a function in the global map, which returns the whole tsdf data as a vector
[x] instead of using the localmap to generate a tsdf marker for ros, use the global maps functionality.

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
[ ] Write and read IntersectionStatus to / from HDF5
[ ] Bresenham: precalc of thee finish vertices not necessary, just use the direction vector and do an inbounds() check with the localmap
[ ] Implement the ray tracer as a singleton, so the rays don't need initialization every time. (insert this into masters thesis)
    -> There needs to be an additional array (e.g. init_rays)
    -> function to destroy current instance and make available for reinstanceiation
[ ] Update local and global map with a feature, which allows writing the Intersection status to the hdf5 and reading it

## TODO's ##

[ ] read paper on loop closure, write down most important points
[ ] Re-evaluate cleanup artifacts method from global map
[ ] when updating the cells later on, possibly favor more recent positions over old ones (sinus/cosinus function)

[ ] write a job, which removes the cells, which have not been covered during bresenham/raytracing (might be too runtime excessive)
[ ] Write updated path back to h5
[ ] think of a way to include connectivity between cells in the update process, so that connecting cells wont be ripped apart as much
    -> somehow keep more of the connectivity
[ ] write a raytracer routine, which removes artifacts not seen during the tracing process (aka keep going even once finished and mark hit cells to be
    removed)
[ ] instead of filtering outliers using the localmap interface, do it via the globalmap, this should be a lot faster

## ASAP ##
[ ] Integration of GTSAM for Loop Closure optimization: https://gtsam.org/tutorials/intro.html
[ ] Map update: calculate the new cell pos for one pose change, not for all associated ones. this simplifys stuff. Do multiple functions for all this.
[ ] as of now, in the map update, only poses are considered, which have been updated, though - what about the ones, which have not been updated, but
    may still be associated with the cell we want to move. because the pose itself has been practically unchanged, it also needs to be considered, when finding a new cell position.


    Also, the loop closure does not only update the path poses between the start and end of the path, but finds new optimal poses for the whole path.

[ ] Fix LC with liosam information
[ ] fix lc detection by using the optimal lc found
[ ] edge case: find bounding box (no axis aligned) of the tsdf volume, find overlap(s) with rest of the map
    -> the data in the overlap(s) is basically broken and cannot be restored, because no sufficient data is present
[ ] That's why the best way might actually be to use the incredibly greedy way and store the pcl data for each of the poses and redo the whole map when finding a lc
[ ] -> basically applying the lc as a graph slam solution (with the data present), generating the new tsdf based on the new poses and their pcl data



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