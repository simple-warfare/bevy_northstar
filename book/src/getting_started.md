# Introduction

[`bevy_northstar`](https://github.com/jtothethree/bevy_northstar) is a [`Hierarchical Pathfinding`](https://alexene.dev/2019/06/02/Hierarchical-pathfinding.html) plugin for [Bevy](https://bevy.org/).

The crate provides:

* Pathfinding Grids: A grid represents the walkable space used for the pathfinding alogrithsm and also precalculated chunks, entrances, and internal paths used for HPA*.

* Pathfinding Systems: Bevy systems to handle pathfinding and collision for you. They are not required to use HPA* or other pathfinding algorithms.

* Pathfinding Algorithms: You can call the pathfinding functions easily if you desire to handle the pathfinding logic in your own systems.

* Debugging Tools: Easily visualize the grid and calculated paths to troubleshoot any tilemap and pathfinding issues.

The crate is currently designed for use with 2d and 3d grid based tilemaps. It is not dependent on any specific tilemap Bevy crate, though it's been designed for ease of use with [`bevy_ecs_tilemap`](https://github.com/StarArawn/bevy_ecs_tilemap) and any related crates such as [`bevy_ecs_tiled`](https://github.com/adrien-bon/bevy_ecs_tiled) and [`bevy_ecs_ldtk`](https://github.com/Trouv/bevy_ecs_ldtk).

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
    commands.spawn(Grid<CardinalNeighborhood>::new(&GridSettings {
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

## Grid Generic Neighborhood Shorthand Types
The following shorthand types are also available for constructing and referencing a `Grid::<N>`.

* CardinalGrid
* CardinalGrid3d
* OrdinalGrid
* OrdinalGrid3d

Rather than `Grid<CardinalNeighborhood>::new` you can use `CardinalGrid::new`.
Rather than querying `Single<&mut Grid<CardinalNeighborhood>>` you can query `Single<&mut CardinalGrid>`

## Grid as a Component
Currently the plugin Pathfinding and Debug systems expect there to be only a single copy of the Grid component which means you can't currently use them if you want to support multiple grids in your project. 

Normally it would make sense for this to be Bevy `Resource` but this decision was made so the plugin can update to support multiple grids in the future without making breaking API changes. If your project needs to support multiple pathfinding grids you can avoid using the NorthstarPlugin and NorthstarDebugPlugin and call the pathfinding functions directly on the `Grid` components for the time being.
