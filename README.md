# bevy_northstar
## A 2d/3d hierarchical pathfinding crate for Bevy. 

`bevy_northstar` works by dividing the map into chunks and then calculates nodes based on the entrances between chunks. The nodes are used in pathfinding to get a higher level path that is significantly faster to calculate over long distances. Once the high level path is determined between a start and goal point it's refined to get a more accurate path.

### This crate is still a work in progress. The base functionality is in place, there's still a bug I'm hunting down in the demo with some npcs having issues pathfinding.

The crate is currently opinionated in the sense that it's not bring-your-own-grid. That may change in the future.

## Demo
cargo run --example demo

Press P to switch between HPA* and traditional A*

![2024-12-02_08-44](https://github.com/user-attachments/assets/18778c4e-43bf-4e4c-8031-8a5974610f9c)

## Current features
* Works with 2d and 3d tilemaps
* Algorithms have been heavily benchmarked
* Gizmo debug view for Bevy
* 2D Demo

## TODO:
* Add dynamic "colliders" to factor into path finding, there's currently no avoidance for other npcs etc
* Add better integration with Bevy, event/time triggers and systems for dynamic refining paths will allow for greater performance
* Add functions for modifying/rebuilding tiles after they've been built
* 3d performance has performance issues greater than expected
* Add psuedo 3d support for tilemap layers (stairs, ramps) without relying on full 3d calculations
* Parallel building of the grid/graph
* Parallelize the Bevy systems mentioned above
* Add 3d examples

## Assets credits
- [kenny-minimap-pack](https://kenney.nl/assets/minimap-pack): an 8x8 tileset from [Kenney](https://kenney.nl/), licensed under [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/)


## Thanks
Thanks to the following crates and blogs that have been used as references
* https://github.com/evenfurther/pathfinding
* https://github.com/mich101mich/hierarchical_pathfinding
* https://alexmelenchon.github.io/Hierarchial-Pathfinding-Research/
