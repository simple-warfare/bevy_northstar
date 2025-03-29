
use bevy::{color::palettes::css, math::UVec3, prelude::{Color, Component}, reflect::Reflect, transform::components::Transform};

use crate::debug::MapType;

/// Grid position
#[derive(Component, Default, Debug, Clone, Eq, PartialEq, Hash)]
pub struct Position(pub UVec3);

/****************************************
    PATHFINDING COMPONENTS
*****************************************/

/// Insert this component to an entity to have the plugin systems
/// pathfind to the goal position
/// 
/// # Fields
/// 
/// * `goal` - The goal position to pathfind to
/// * `use_astar` - Use A* pathfinding algorithm. Use this to bypass HPA*.
#[derive(Component, Default, Debug)]
pub struct Pathfind {
    pub goal: UVec3,
    pub use_astar: bool,
}

/// The next position in the path. This is inserted every frame
/// by the plugin systems after it's exisiting `Next' componen is removed.`
#[derive(Component, Default, Debug)]
pub struct Next(pub UVec3);

// See src/path.rs for the Path component

/****************************************
    COLLISION COMPONENTS
*****************************************/

/// Collision marker for moving entities
#[derive(Component, Default)]
pub struct Blocking;

/// Marker component that is inserted to an entity 
/// when local avoidance fails
#[derive(Component, Default, Debug)]
pub struct AvoidanceFailed;

/// Marker component that is inserted to an entity
/// when rerouting a path fails
#[derive(Component, Default, Debug)]
pub struct RerouteFailed;

/****************************************
    DEBUGGING COMPONENTS
*****************************************/

/// Component for debugging an entity's pathfinding
#[derive(Component)]
pub struct DebugPath {
    pub tile_width: u32,
    pub tile_height: u32,
    pub map_type: MapType,
    pub color: Color,
    pub draw_unrefined: bool,
}

impl Default for DebugPath {
    fn default() -> Self {
        DebugPath {
            tile_width: 16,
            tile_height: 16,
            map_type: MapType::Square,
            color: bevy::prelude::Color::Srgba(css::RED),
            draw_unrefined: false,
        }
    }
}

/// Component for setting the gizmo colors for an entity
#[derive(Component, Default)]
pub struct DebugColor(pub Color);

/// Component for debugging the map.
/// Insert this component as a child of your map.
/// Use the transform component if you need to position the debug map.
/// 
/// # Fields
/// 
/// * `tile_width` - The width of a tile in pixels
/// * `tile_height` - The height of a tile in pixels
/// * `map_type` - The type of map `MapType` to debug
/// * `draw_chunks` - Debug the chunks of the map
/// * `draw_points` - Debug the points of the map, this is intensive
/// * `draw_entrances` - Debug the entrances of the map
/// * `draw_cached_paths` - Debug the cached paths of the map
/// 
#[derive(Reflect, Component)]
#[require(Transform)]
pub struct DebugMap {
    pub tile_width: u32,
    pub tile_height: u32,
    pub map_type: MapType,
    //
    pub draw_chunks: bool,
    pub draw_points: bool,
    pub draw_entrances: bool,
    pub draw_cached_paths: bool,
}

impl Default for DebugMap {
    fn default() -> Self {
        DebugMap {
            tile_width: 16,
            tile_height: 16,
            map_type: MapType::Square,
            draw_chunks: true,
            draw_points: false,
            draw_entrances: false,
            draw_cached_paths: false,
        }
    }
}