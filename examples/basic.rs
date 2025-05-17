use bevy::prelude::*;
use bevy_northstar::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // Add the Northstar Plugin with a selected neighborhood to use the built in pathfinding systems
        .add_plugins(NorthstarPlugin::<OrdinalNeighborhood>::default())
        .run();
}


