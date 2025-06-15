# bevy_northstar
[![Crates.io](https://img.shields.io/crates/v/bevy_northstar)](https://crates.io/crates/bevy_northstar)
[![docs](https://docs.rs/bevy_northstar/badge.svg)](https://docs.rs/bevy_northstar/)
[![license](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/jtothethree/bevy_northstar/blob/main/LICENSE)
[![Crates.io](https://img.shields.io/crates/d/bevy_northstar)](https://crates.io/crates/bevy_northstar)
[![Following released Bevy versions](https://img.shields.io/badge/Bevy%20tracking-released%20version-lightblue)](https://bevyengine.org/learn/quick-start/plugin-development/#main-branch-tracking)
## A 2d/3d hierarchical pathfinding crate for Bevy. 

`bevy_northstar` works by dividing the map into chunks and then calculates nodes based on the entrances between chunks. The nodes are used in pathfinding to get a higher level path that is significantly faster to calculate over long distances. Once the high level path is determined between a start and goal point it's refined to get a more accurate path.
The crate provides:

## Features  
- **Supports 2D and 3D Tilemaps** – Supports 2d and 3d tilemaps.  

- **Optimized Performance** – Algorithms are heavily benchmarked for efficiency.  

- **Gizmo Debug View** – Debug visuals for verifying the built HPA graph. Pathing debug components to visualize an entities path.  

- **Stress Tests** – 128x128 map with 128 entities to stress test HPA vs A* and collision. A collision example is provided to stress test narrow pathing.

- **Dynamic Collision & Avoidance** – For moving colliders attaching a simple Blocking marker component is all that's needed. If you use the built in systems the pathing will do a configurable look ahead to see if it can do a fast local A* reroute.

- **Bevy Systems Integration** – Bevy systems and components for pathfinding as well as collision markers when avoidance paths fail.



## Demo
cargo run --example demo --features stats --release

Press P to switch between HPA* and traditional A*
Press C to disable/enable collision

![2024-12-02_08-44](https://github.com/user-attachments/assets/18778c4e-43bf-4e4c-8031-8a5974610f9c)

## Feature Flags
This crate has the following Cargo features:

- `stats`: Enables pathfinding benchmarks. Useful to get an idea of how much time it's using per frame.

# Quick Start

Add required dependencies to your `Cargo.toml` file:

```toml
[dependencies]
bevy = "0.16"
bevy_northstar = "0.2"
```

The basic requirements to use the crate are to spawn an entity with a `Grid` component, adjust the points, and then call `Grid::build()` so the chunk entrances and internal paths can be calculated. 

To use the built-in pathfinding systems for the crate, insert the NorthstarPlugin specifying the `Neighborhood` to use.

The built-in neighborhoods are:
* `CardinalNeighborhood` 4 directions allowing no diagonal movement.
* `CardinalNeighborhood3d` 6 directions, including up and down, allowing no diagonal movement.
* `OrdinalNeighborhood` 8 directions allowing for diagonal movement.
* `OrdinalNeighborhood3d` 26 directions which includes the base ordinal movements and their up/down directions.

```rust,no_run
use bevy::prelude::*;
use bevy_northstar::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // Add the Northstar Plugin with a selected neighborhood to use the built in pathfinding systems
        .add_plugins(NorthstarPlugin::<CardinalNeighborhood>::default())
        .add_systems(Startup, (startup, build_grid.after(startup)))
        .run();
}

fn startup(mut commands: Commands) {
    commands.spawn(Camera2d::default());

    // Spawn the grid used for pathfinding.
    commands.spawn(Grid::<CardinalNeighborhood>::new(&GridSettings {
        width: 16,
        height: 16,
        chunk_size: 4,
        ..Default::default()
    }));
}

fn build_grid(grid: Single<&mut Grid<CardinalNeighborhood>>) {
    let mut grid = grid.into_inner();

    // Let's set the position 8, 8 to a wall
    grid.set_point(UVec3::new(8, 8, 0), Point::new(u32::MAX, true));

    info!("Building the grid...");

    // The grid needs to be built after setting the points.
    // Building the grid will calculate the chunk entrances and cache internal paths.
    grid.build();

    info!("Grid built successfully!");
}
```

## Bevy Compatibility

|bevy|bevy_northstar|
|---|---|
|0.16|0.2|


## Roadmap / TODO
- **Modify & Rebuild Grid Chunks Dynamically** – Support updates to the grid after it’s been built.    
- **Pseudo-3D Tilemap Support** – Add support for features like stairs and ramps without full 3D calculations.  
- **Parallelized Graph Building** – Speed up grid/graph construction using parallelism.  
- **Add Support For Multiple HPA Levels** – Implement multiple hierarchical levels for improved efficiency.  
- **Optimize 3D Performance** – 3d grids appear to take a performance hit higher than expected currently. 

## Assets credits
- [kenny-minimap-pack](https://kenney.nl/assets/minimap-pack): an 8x8 tileset from [Kenney](https://kenney.nl/), licensed under [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/)


## Thanks
Thanks to the following crates and blogs that have been used as references
* <https://github.com/evenfurther/pathfinding>
* <https://github.com/mich101mich/hierarchical_pathfinding>
* <https://alexmelenchon.github.io/Hierarchial-Pathfinding-Research/>
