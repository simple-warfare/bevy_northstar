# Debugging the Grid component

The `NorthstarDebugPlugin` adds systems that can be enabled to draw gizmos to display the following:

* Chunk grid: Grid outline of where the chunks are
* Entrances: Calculated entrances at the boundaries of the chunks
* Cached internal paths: Cached paths inside the chunk between its own entrances.
* Cells: Each individual cell on the grid and its passable status
* Paths: Draws the current calculated path components

First, add the `NorthstarDebugPlugin` to your app.

Then insert the `DebugGrid` component a child of the entity that the `Grid` you want to debug is attached to. 

You will likely also want to add a `DebugOffset` component to the same entity as `DebugGrid` to align the gizmos with your world tilemap position. In this example we'll be using components from bevy_ecs_tilemap to assist with determining our offset.



```rust,no_run
use bevy::prelude::*;
use bevy_ecs_tilemap::prelude::*;
use bevy_northstar::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(NorthstarPlugin::<CardinalNeighborhood>::default())
        // Adding the NorthstarDebugPlugin
        // You need to specify the neighborhood here as well
        .add_plugins(NorthstarDebugPlugin::<CardinalNeighborhood>::default())
        .add_systems(Startup, startup)
        .run();
}

fn startup(mut commands: Commands, asset_server: Res<AssetServer>) {
    // Query or create your bevy_ecs_tilemap TilemapAnchor
    let anchor = TilemapAnchor::Center;

    // Query or create your bevy_ecs_tilemap TilemapSize, TilemapGridSize, and TilemapTileSize. They are required for calculating the anchor offset.
    let tilemap_size = TilemapSize { x: 64, y: 64 };
    let tilemap_gridsize = TilemapGridSize { x: 8.0, y: 8.0 };
    let tilemap_tilesize = TilemapTileSize { x: 8.0, y: 8.0 };

    // Store our offset
    let offset = anchor.as_offset(
        &tilemap_size,
        &tilemap_gridsize,
        &tilemap_tilesize,
        &TilemapType::Square,
    );

    // Let's pretend we loaded a tilemap asset and stored it in a handle and now we're spawning the entity for it.
    let mut map_entity = commands.spawn(anchor);

    // Call `build()` to return the component.
    let debug_grid = DebugGridBuilder::new(64, 64)
        .enable_chunks()
        .enable_entrances()
        .build();


    // Spawn an entity with the Grid and DebugMap component as a child of the map entity.
    let grid_entity = map_entity.with_child(
        CardinalGrid::new(&GridSettingsBuilder::new_2d(64, 64).build())
    );

    grid_entity.with_child((
        debug_grid,
        DebugOffset(offset)
    ));
}
```

## DebugGridBuilder Settings

### `isometric()`

Sets the debug gizmos to draw in isometric perspective.

### `enable_chunks()`

Outline the grid chunk regions.

### `enable_entrances()`

Highlights the entrances created between each chunk. Very useful for debugging HPA* issues.

### `enable_cells()`

Overlay over each tile whether it's passable or impassable. Useful for debugging if you're calling `set_nav` correctly for your tilemap. 

### `enable_cached_paths()`

`Grid` precaches paths between all entrances inside each chunk. Noisy, but can help debug HPA* pathing issues.

# Debugging Paths

Debugging paths for an entity requires you to add a `DebugPath` component to the entity or entities of your choosing. This component allows you to selectively debug specific paths.

Drawing the path gizmos also requires the `NorthstarDebugPlugin` to add the gizmo drawing system.

```rust,no_run
use bevy::prelude::*;
use bevy_northstar::prelude::*;

commands.spawn((
    Name::new("Player"),
    DebugPath::new(Color::srgb(1.0, 0.0, 0.0)),
));
```

If you would like to debug a directly created path (returned from `grid::pathfind()`) make sure you attach the returned `Path` component to your entity. If you're not using `NorthstarPlugin` you will also need to make sure the entity has an `AgentPos` component. This is the query filter used to debug paths `Query<(&DebugPath, &Path, &AgentOfGrid)>`.

# Stats
Enabling the `stats` feature on the crate will allow the `NorthstarPlugin` pathfinding systems to calculate the average time spent on pathfinding and collision calls.

```toml
[dependencies]
bevy_northstar = { version = "0.2.0", features = ["stats"]}
```

You can access the statistics from the `Stats` `Resource`.

```rust,no_run
fn print_stats(stats: Res<Stats>) {
    debug!("Collision Stats: {:?}", stats.collision);
    debug!("Pathfinding Stats: {:?}", stats.pathfinding);
}
```