# bevy_northstar
## A 2d/3d hierarchical pathfinding crate for Bevy. 

`bevy_northstar` works by dividing the map into chunks and then calculates nodes based on the entrances between chunks. The nodes are used in pathfinding to get a higher level path that is significantly faster to calculate over long distances. Once the high level path is determined between a start and goal point it's refined to get a more accurate path.

### This crate is still a work in progress.

The crate is currently opinionated in the sense that it's not bring-your-own-grid. That may change in the future.

## Demo
cargo run --example demo --features stats --release

Press P to switch between HPA* and traditional A*
Press C to disable/enable collision

![2024-12-02_08-44](https://github.com/user-attachments/assets/18778c4e-43bf-4e4c-8031-8a5974610f9c)


## Features  
✅ **Supports 2D and 3D Tilemaps** – Supports 2d and 3d tilemaps.  

✅ **Optimized Performance** – Algorithms are heavily benchmarked for efficiency.  

✅ **Gizmo Debug View** – Debug visuals for verifying the built HPA graph. Pathing debug components to visualize an entities path.  

✅ **Stress Tests** – 128x128 map with 128 entities to stress test HPA vs A* and collision. A collision example is provided to stress test narrow pathing.

✅ **Dynamic Collision & Avoidance** – For moving colliders attaching a simple Blocking marker component is all that's needed. If you use the built in systems the pathing will do a configurable look ahead to see if it can do a fast local A* reroute.

✅ **Bevy Systems Integration** – Bevy systems and components for pathfinding as well as collision markers when avoidance paths fail.

## Roadmap / TODO  
🚀 **Next Steps:**  
- [ ] **Code & Documentation Cleanup** – Refine and document the API.
- [ ] **Basic Example** - Create a minimal example demonstrating only the basics usage for the crate. Move existing examples to stress tests.
- [ ] **Release Beta Version** - Publish a beta version once the API is in a good place.

🔧 **Planned Features & Improvements:**  
- [ ] **Modify & Rebuild Grid Chunks Dynamically** – Support updates to the grid after it’s been built.    
- [ ] **Pseudo-3D Tilemap Support** – Add support for features like stairs and ramps without full 3D calculations.  
- [ ] **Parallelized Graph Building** – Speed up grid/graph construction using parallelism.  
- [ ] **Add Support For Multiple HPA Levels** – Implement multiple hierarchical levels for improved efficiency.  
- [ ] **Optimize 3D Performance** – 3d grids appear to take a performance hit higher than expected currently. 

## Assets credits
- [kenny-minimap-pack](https://kenney.nl/assets/minimap-pack): an 8x8 tileset from [Kenney](https://kenney.nl/), licensed under [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/)


## Thanks
Thanks to the following crates and blogs that have been used as references
* https://github.com/evenfurther/pathfinding
* https://github.com/mich101mich/hierarchical_pathfinding
* https://alexmelenchon.github.io/Hierarchial-Pathfinding-Research/
