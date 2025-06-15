//! Components for pathfinding and collision
use bevy::{
    color::palettes::css,
    math::UVec3,
    prelude::{Color, Component},
    reflect::Reflect,
    transform::components::Transform,
};

use crate::debug::DebugMapType;

/// An entities position on the grid.
/// You'll need to maintain this position if you use the plugin pathfinding systems.
#[derive(Component, Default, Debug, Clone, Eq, PartialEq, Hash)]
pub struct GridPos(pub UVec3);

/****************************************
    PATHFINDING COMPONENTS
*****************************************/

/// Insert `Pathfind` on an entity to pathfind to a goal.
/// Once the plugin systems have found a path, [`NextPos`] will be inserted.
#[derive(Component, Default, Debug)]
pub struct Pathfind {
    /// The goal to pathfind to.
    pub goal: UVec3,
    /// Set to true to bypass HPA* and use a direct A* search.
    pub use_astar: bool,
}

impl Pathfind {
    /// Creates a new `Pathfind` component with the given goal. HPA* will be used to pathfind.
    pub fn new(goal: UVec3) -> Self {
        Pathfind {
            goal,
            use_astar: false,
        }
    }

    /// Creates a new `Pathfind` component with the given goal. Traditional A* will be used to pathfind.
    pub fn new_astar(goal: UVec3) -> Self {
        Pathfind {
            goal,
            use_astar: true,
        }
    }
}

/// The next position in the path.
/// The `pathfind` system will insert this into the entity when a path is found.
/// Remove `NextPos` after you've moved the entity to the next position and
/// a new `NextPos` will be inserted.
#[derive(Component, Default, Debug)]
pub struct NextPos(pub UVec3);

// See src/path.rs for the Path component

/****************************************
    COLLISION COMPONENTS
*****************************************/

/// Collision marker for moving entities.
/// This is used for moving enities to check for collisions and shouldn't be used for
/// static blocking tiles such as walls.
#[derive(Component, Default)]
pub struct Blocking;

/// Marker component that is inserted to an entity when a collision is detected.
/// The built-in pathfinding system will try to pathfind for this entity every frame unless
/// you handle the failure in a custom way.
#[derive(Component, Default, Debug)]
pub struct PathfindingFailed;

/// Marker component that is inserted to an entity when local avoidance fails.
/// Currently this marker is handled by the `reroute_path` system and can be ignored
/// unless the desire is to handle the failure in a custom way.
#[derive(Component, Default, Debug)]
pub struct AvoidanceFailed;

/// Marker component that is inserted to an entity when path rerouting in `reroute_path` fails.
/// You will need to handle this failure before the entity will be pathed again.
#[derive(Component, Default, Debug)]
pub struct RerouteFailed;

/****************************************
    DEBUGGING COMPONENTS
*****************************************/

/// Component for debugging an entity's pathfinding.
#[derive(Component)]
pub struct DebugPath {
    /// The width of a tile in pixels.
    pub tile_width: u32,
    /// The height of a tile in pixels.
    pub tile_height: u32,
    /// Tilemap layout [`DebugMapType`] for determinining gizmo placement.
    pub map_type: DebugMapType,
    /// The [`Color`] of the path gizmo.
    pub color: Color,
    /// Draw the raw HPA* path before it's been refined.
    /// This is useful for debugging the HPA* algorithm.
    pub draw_unrefined: bool,
}

impl Default for DebugPath {
    fn default() -> Self {
        DebugPath {
            tile_width: 16,
            tile_height: 16,
            map_type: DebugMapType::Square,
            color: bevy::prelude::Color::Srgba(css::RED),
            draw_unrefined: false,
        }
    }
}

/// Component for setting the debug gizmo colors for an entity.
#[derive(Component, Default)]
pub struct DebugColor(pub Color);

/// Component for debugging the map.
/// Insert `DebugMap` as a child of your map.
#[derive(Reflect, Component)]
#[require(Transform)]
pub struct DebugMap {
    /// The width of a tile in pixels.
    pub tile_width: u32,
    /// The height of a tile in pixels.
    pub tile_height: u32,
    /// Tilemap layout [`DebugMapType`] for determinining gizmo placement.
    pub map_type: DebugMapType,
    /// Draw the chunk boundaries.
    pub draw_chunks: bool,
    /// Draw each point of the map, useful for debugging tilemap issues.
    pub draw_points: bool,
    /// Draws the connections used by the HPA* algorithm between the chunks.
    pub draw_entrances: bool,
    /// Draws the internal cached paths between the entrances in the same chunk.
    /// This is only really useful for debugging issues with the HPA* crate.
    pub draw_cached_paths: bool,
}

impl Default for DebugMap {
    fn default() -> Self {
        DebugMap {
            tile_width: 16,
            tile_height: 16,
            map_type: DebugMapType::Square,
            draw_chunks: true,
            draw_points: false,
            draw_entrances: false,
            draw_cached_paths: false,
        }
    }
}
