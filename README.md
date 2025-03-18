# bevy_northstar
## A 2d/3d hierarchical pathfinding crate for Bevy. 

`bevy_northstar` works by dividing the map into chunks and then calculates nodes based on the entrances between chunks. The nodes are used in pathfinding to get a higher level path that is significantly faster to calculate over long distances. Once the high level path is determined between a start and goal point it's refined to get a more accurate path.

### This crate is still a work in progress.

The crate is currently opinionated in the sense that it's not bring-your-own-grid. That may change in the future.

## Demo
cargo run --example demo --features stats

Press P to switch between HPA* and traditional A*
Press C to enable collision

![2024-12-02_08-44](https://github.com/user-attachments/assets/18778c4e-43bf-4e4c-8031-8a5974610f9c)


## Features  
✅ **Supports 2D and 3D Tilemaps** – Works seamlessly with both 2D and 3D grids.  
✅ **Optimized Performance** – Algorithms are heavily benchmarked for efficiency.  
✅ **Gizmo Debug View** – Built-in visualization using Bevy's gizmo system.  
✅ **2D Demo Included** – Working example with profiling stats and collision.  
✅ **Dynamic Collision & Avoidance** – Uses a hashmap to efficiently update moving obstacles.  
✅ **Bevy Systems Integration** – Bevy systems and components for pathfinding.

## Roadmap / TODO  
🚀 **Next Steps:**  
- [ ] **Code & Documentation Cleanup** – Refine and document the API.  
- [ ] **Initial Crate Release** – Publish the first version.  

🔧 **Planned Features & Improvements:**  
- [ ] **Modify & Rebuild Tiles Dynamically** – Support updates to the tilemap after it’s been built.  
- [ ] **Async Pathfinding Support** – Enable async functions for avoidance cases where full path recalculations are needed for frame stability.  
- [ ] **Hierarchical Pathfinding** – Implement multiple hierarchical levels for improved efficiency.  
- [ ] **Optimize 3D Performance** – 3d grids appear to take a performance hit higher than expected currently.  
- [ ] **Pseudo-3D Tilemap Support** – Add support for features like stairs and ramps without full 3D calculations.  
- [ ] **Parallelized Graph Building** – Speed up grid/graph construction using parallelism.  
- [ ] **Parallel Bevy Systems** – Optimize pathfinding system execution where possible.  
- [ ] **3D Examples & Demos** – Provide additional examples for 3D use cases.  

## Assets credits
- [kenny-minimap-pack](https://kenney.nl/assets/minimap-pack): an 8x8 tileset from [Kenney](https://kenney.nl/), licensed under [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/)


## Thanks
Thanks to the following crates and blogs that have been used as references
* https://github.com/evenfurther/pathfinding
* https://github.com/mich101mich/hierarchical_pathfinding
* https://alexmelenchon.github.io/Hierarchial-Pathfinding-Research/
