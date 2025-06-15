# Debugging the Grid component

The `NorthstarDebugPlugin` adds systems that can be enabled to draw gizmos to display the following:

* Chunk grid: Grid outline of where the chunks are
* Entrances: Calculated entrances at the boundaries of the chunks
* Cached internal paths: Cached paths inside the chunk between its own entrances.
* Points: Each individual point on the grid and it's walkable status
* Paths: Draws the current calculated path components

First, add the `NorthstarDebugPlugin` to your app.

Second, you will likely want to set the transform offset to align it with your tilemap. In this example we'll be using components from bevy_ecs_tilemap to assist with determining our offset.

Third, insert the `DebugMap` component to the same entity as your `Grid`. Currently doing this won't change anything, but in the future the Plugin will likely use that to match up `DebugMap` with it's `Grid` when multiple `Grid`s are supported.

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
    let tilemap_size = TilemapSize { x: 128, y: 128 };
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

    // Spawn an entity with the Grid and DebugMap component as a child of the map entity.
    map_entity.with_child(
        CardinalGrid::new(&GridSettings {
            width: 128,
            height: 128,
            ..default::Default()
        }),
        DebugMap {
            tile_width: 8,
            tile_height: 8,
            map_type: DebugMapType::Square,
            draw_chunks: true,
            draw_points: false,
            draw_entrances: true,
            draw_cached_paths: false,
        },
        // Use our offset calculated from the bevy_ecs_tilemap anchor to set the transform of the DebugMap entity.
        Transform::from_translation(offset.extend(0.0))
    )
}
```

# Debugging Paths

Debugging calculated paths for an entity requires you to add a `DebugPath` component to the entity or entities of your choosing. This component allows you to selectively debug specific paths.

Drawing the path gizmos also requires the `NorthstarDebugPlugin` to add the gizmo drawing system.

```rust,no_run
use bevy::prelude::*;
use bevy_northstar::prelude::*;

commands.spawn((
    Name::new("Player"),
    DebugPath {
        // Width of the tilemap tiles
        tile_width: 8,
        // Height of the tilemap tiles
        tile_height: 8,
        // Map Type (Square or Isometric)
        map_type: DebugMapType::Square,
        // Color of the path gizmos
        color: Color::srgb(1.0, 0.0, 0.0)
    },
))
```

# Stats
Enabling the `stats` feature on the crate will allow the `NorthstarPlugin` pathfinding systems to calculate how much time is spent per frame on pathfinding and collison.

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