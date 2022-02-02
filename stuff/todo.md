## TODO ##

* use globalmap and localmap from fastsense, adapt everything needed  !!DONE!!
* check the functionality of the globalmap to create a map from file  !!DONE SELFMADE!!
* start using the maps efficently !! WIP !!
* create a method in the localmap, which checks the current intersection of a ray with the localmap
* create a way ( in the localmap ) to read, which part of the globalmap can be seen from a certain position
* read this part
* find a way to display intersections in a nice manner (e.g. some weird weight or a new variable) !! WIP !!

### 02.02.2022 ###

* read paper on loop closure, write down most important points
* fix bugs in intersections
* get a better understanding on weights (using the original doc thesis)
* possibility to do ray tracing from different positions
* define the data mapping problem -> which "data" comes from which scan? and: where do we actually put it kekw?
* define a way to serialize the path to a hdf5 possibly


## Important ##

* Proof of conzept -> using simulated data and artifical loop closure [to checko algorithm validity]

# Algorithm #

## Loop Closure detection ##

-> use an existing algorithm from hertzberg et al.
-> may need some tweaks for tsdf (matching)
-> if loop is found optimize the pose graph using their method.

## Data association ##
### Intuitive, greedy approach ###

Backwards Iterate from the last known position, trace a artificial laserscan, check for collisions.
For every SCAN s with collisions col:
    For each collision in col:
        1. MARK cell belonging to the collision as associated with the current scan s
        2. REMOVE cell from being part in further collisions
Now (hopefully) every cell belongs to exactly one scanning position, thus we solved the data mapping problem in TSDF the most easy way:
We assigned every cell to the scan, which has seen ist LAST, this might be not the optimal solution, but may work as a very greedy base to
see where we are going with this.

The psuedo code contains multiple pitfalls (Words printed in FAT need to be evaluated and specified much further):

1. SCAN -> artificial laserscanning "raytracing" from the scanning positions to solve data association problem
    * apply from every single position of the pose graph or discretize, if so, which discretization may be good?
    * the criteria for when a single ray of the scan is supposed to be stopped is very important, as we are only interested in the first collision with an
      object, thus the ray is explicitly cancelled if it hits cells, which draw the conclusion, that we are not only already behind an object (so in an area which can definetly not be seen by an ordinary laserscanner), but also out of bounds of the TSDF, which describes the near surface area
    * there needs to be a way to track collisions, e.g. with a data structure or something similar.

2. MARK -> somehow mark the cell which is currently taking part in a collision, adding it to a string or doing whatever, occupancy map?
3. REMOVE (only for this greedy approach) -> we want to create a 1:1 data association, thus the currently colliding cell should not be considered in
   furhter collisions.

### Computitionally and memory intensive apporach ###

-> Don't even try to do an 1:1 data association, do a 1:N => 1 Cell to many Poses
-> Calculate the new position by using some sort of a weighted average or flying average of the positions the cell would have when being moved
   according to each's pose. Might be tricky as a string

### Furhter approaches ###

Approach for saving the associated data:

- in a string: #Pose + LocalMapIndex   -> AWESOME if we want to compare different association approaches (because they can be repeated easily)

## Global Map optimization ##

### Loop Closure Optimization ###
-> After a loop is detected and we solved the data association problem AND the pose graph is optimized,
we have to update the globalmap.

e.g.: when a position in the posegraph is slightly translated and rotated, the same needs to happen for the TSDF data associated
      with this position. This might lead to OVERLAP. It needs to be thought of, how this overlap is dealt with

### Post Loop Closure Optimization ###

After the loop closure adaptions, there may be some problems with the TSDF, e.g. it might be ripped apart in corners etc., this needs to be 
evaluated and possibly fixed