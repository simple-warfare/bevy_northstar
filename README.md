# bevy_northstar
## A 2d/3d hierarchical pathfinding crate for Bevy. 

`bevy_northstar` works by dividing the map into chunks and then calculates nodes based on the entrances between chunks. The nodes are used in pathfinding to get a higher level path that is significantly faster to calculate over long distances. Once the high level path is determined between a start and goal point it's refined to get a more accurate path.

### This crate is still a work in progress. Most of the base functionality is still there but has a few bugs in it.

The crate is currently opinionated in the sense that it's not bring-your-own-grid. That may change in the future.

## Current features
* Works with 2d and 3d tilemaps
* Algorithms have been heavily benchmarked
* Gizmo debug view for Bevy

## TODO:
Fix bugs found in some instances with bad edges
Add dynamic "colliders" to factor into path finding, there's currently no avoidance for other npcs etc
Add better integration with Bevy, event/time triggers and systems for dynamic refining paths will allow for greater performance
Add functions for modifying/rebuilding tiles after they've been built
3d performance has performance issues greater than expected
Add psuedo 3d support for tilemap layers (stairs, ramps) without relying on full 3d calculations
Parallel building of the grid/graph
Parallelize the Bevy systems mentioned above
Add 2d and 3d examples


### Credits
Credit to the following crates and blogs that have been used as references
https://github.com/evenfurther/pathfinding
https://github.com/mich101mich/hierarchical_pathfinding
https://alexmelenchon.github.io/Hierarchial-Pathfinding-Research/
