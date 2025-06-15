// This is a bare minimum example for setting up the plugin.
// Nothing is displayed on run.

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
    commands.spawn(CardinalGrid::new(&GridSettings {
        width: 16,
        height: 16,
        chunk_size: 4,
        ..Default::default()
    }));
}

fn build_grid(grid: Single<&mut CardinalGrid>) {
    let mut grid = grid.into_inner();

    // Let's set the position 8, 8 to a wall
    grid.set_point(UVec3::new(8, 8, 0), Point::new(u32::MAX, true));

    info!("Building the grid...");

    // The grid needs to be built after setting the points.
    // Building the grid will calculate the chunk entrances and cache internal paths.
    grid.build();

    info!("Grid built successfully!");
}