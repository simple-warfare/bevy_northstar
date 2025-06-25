//! This module contains the `Grid` struct which is the primary `Resource` for the crate.
use bevy::{
    math::{IVec3, UVec3},
    platform::collections::HashMap,
    prelude::{Component, Entity},
};
use ndarray::{Array3, ArrayView2, ArrayView3};

use crate::{
    chunk::Chunk,
    dijkstra::*,
    dir::*,
    graph::Graph,
    neighbor::{Neighborhood, ORDINAL_3D_OFFSETS},
    node::Node,
    path::Path,
    pathfind::{pathfind, pathfind_astar, reroute_path},
};

/// Settings for dividing the grid into chunks.
#[derive(Copy, Clone, Debug)]
pub struct ChunkSettings {
    /// The square size of each chunk in the grid.
    /// Needs to be at least 3.
    pub size: u32,
    /// The depth of each chunk in the grid when using 3D grids.
    pub depth: u32,
    /// If true, allows corner connections between chunks.
    /// This will increase the time it takes to build the grid.
    /// It generally isn't recommended as the path refinement step should handle corners already unless you have a noisy tilemap.
    pub diagonal_connections: bool,
}

impl Default for ChunkSettings {
    fn default() -> Self {
        ChunkSettings {
            size: 16,
            depth: 1,
            diagonal_connections: false,
        }
    }
}

/// Defaults for initializing the grid points.
#[derive(Copy, Clone, Debug)]
pub struct CostSettings {
    /// The default cost for each point in the grid.
    pub default_cost: u32,
    /// If true, the default points will be solid and block movement.
    pub default_solid: bool,
}

impl Default for CostSettings {
    fn default() -> Self {
        CostSettings {
            default_cost: 1,
            default_solid: false,
        }
    }
}

/// Settings for collision
#[derive(Copy, Clone, Debug)]
pub struct CollisionSettings {
    /// If true, collision avoidance is enabled.
    pub enabled: bool,
    /// The plugin systems use collision avoidance, this is the look ahead for the path to check for blocking entities.
    pub avoidance_distance: u32,
}

impl Default for CollisionSettings {
    fn default() -> Self {
        CollisionSettings {
            enabled: false,
            avoidance_distance: 4,
        }
    }
}

/// Settings for filtering determined neighbors.
#[derive(Copy, Clone, Debug)]
pub struct NeighborhoodSettings {
    /// Prevents diagonal movement through solid corners. It is enabled by default for ordinal neighborhoods.
    /// Enabling this prevents the following movement (x = solid, / = path):
    /// |x|/|
    /// |/|x|
    pub allow_corner_clipping: bool,
    /// Enabling this will prevent diagonal movement around solids.
    /// For real-time movement you may want to enable this if your sprites are larger than a single tile as it will prevent the sprite from clipping through corners.
    /// Will force the following movement (x = solid, / = path):
    /// |o|/|
    /// |/|x|
    /// into:
    /// |-|-|
    /// |||x|
    pub smooth_corner_paths: bool,
}

impl Default for NeighborhoodSettings {
    fn default() -> Self {
        NeighborhoodSettings {
            allow_corner_clipping: false,
            smooth_corner_paths: false,
        }
    }
}

#[derive(Debug)]
pub struct GridSettings(pub(crate) GridInternalSettings);

#[derive(Copy, Clone, Debug)]
pub struct GridSettingsBuilder {
    dimensions: UVec3,
    chunk_settings: ChunkSettings,
    cost_settings: CostSettings,
    collision_settings: CollisionSettings,
    neighborhood_settings: NeighborhoodSettings,
}

impl Default for GridSettingsBuilder {
    fn default() -> Self {
        GridSettingsBuilder {
            dimensions: UVec3::new(64, 64, 1),
            chunk_settings: ChunkSettings::default(),
            cost_settings: CostSettings::default(),
            collision_settings: CollisionSettings::default(),
            neighborhood_settings: NeighborhoodSettings::default(),
        }
    }
}

impl GridSettingsBuilder {
    pub fn new_2d(width: u32, height: u32) -> Self {
        if width < 3 || height < 3 {
            panic!("Width and height must be at least 3");
        }

        let mut grid_settings = GridSettingsBuilder {
            dimensions: UVec3::new(width, height, 1),
            ..Default::default()
        };

        grid_settings.chunk_settings.depth = 1;

        grid_settings
    }

    pub fn new_3d(width: u32, height: u32, depth: u32) -> Self {
        if width < 3 || height < 3 {
            panic!("Width and height must be at least 3");
        }

        if depth < 1 {
            panic!("Depth must be at least 1");
        }

        GridSettingsBuilder {
            dimensions: UVec3::new(width, height, depth),
            ..Default::default()
        }
    }

    pub fn chunk_settings(&mut self, chunk_settings: ChunkSettings) -> Self {
        self.chunk_size(chunk_settings.size);
        self.chunk_depth(chunk_settings.depth);

        if chunk_settings.diagonal_connections {
            self.enable_diagonal_connections();
        }

        *self
    }

    pub fn chunk_size(mut self, chunk_size: u32) -> Self {
        if chunk_size < 3 {
            panic!("Chunk size must be at least 3");
        }

        self.chunk_settings.size = chunk_size;
        self
    }

    pub fn chunk_depth(mut self, chunk_depth: u32) -> Self {
        if chunk_depth < 1 {
            panic!("Chunk depth must be at least 1");
        }

        //FIXME: User should be able to set the chunk depth so there's no z chunking but still allow x/y chunking
        // Right now we need dimension.z to be divisible by chunk_depth
        if self.dimensions.z % chunk_depth != 0 {
            panic!("Depth must be divisible by chunk depth");
        }

        self.chunk_settings.depth = chunk_depth;
        self
    }

    pub fn enable_diagonal_connections(mut self) -> Self {
        self.chunk_settings.diagonal_connections = true;
        self
    }

    pub fn cost_settings(self, cost_settings: CostSettings) -> Self {
        self.default_cost(cost_settings.default_cost);
        
        if cost_settings.default_solid {
            self.default_solid();
        }

        self
    }

    pub fn default_cost(mut self, default_cost: u32) -> Self {
        self.cost_settings.default_cost = default_cost;
        self
    }

    pub fn default_solid(mut self) -> Self {
        self.cost_settings.default_solid = true;
        self
    }

    pub fn enable_collision(mut self) -> Self {
        self.collision_settings.enabled = true;
        self
    }

    pub fn avoidance_distance(mut self, distance: u32) -> Self {
        self.collision_settings.avoidance_distance = distance;
        self
    }

    pub fn collision_settings(mut self, collison_settings: CollisionSettings) -> Self {
        self.collision_settings = collison_settings;
        self
    }

    pub fn neighborhood_settings(mut self, neighborhood_settings: NeighborhoodSettings) -> Self {
        self.neighborhood_settings = neighborhood_settings;
        self
    }

    pub fn allow_corner_clipping(mut self) -> Self {
        self.neighborhood_settings.allow_corner_clipping = true;
        self
    }

    pub fn smooth_corner_paths(mut self) -> Self {
        self.neighborhood_settings.smooth_corner_paths = true;
        self
    }

    pub fn build(self) -> GridSettings {
        GridSettings(GridInternalSettings {
            dimensions: self.dimensions,
            chunk_settings: self.chunk_settings,
            cost_settings: self.cost_settings,
            collision_settings: self.collision_settings,
            neighborhood_settings: self.neighborhood_settings,
        })
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct GridInternalSettings {
    pub(crate) dimensions: UVec3,
    pub(crate) chunk_settings: ChunkSettings,
    pub(crate) cost_settings: CostSettings,
    pub(crate) collision_settings: CollisionSettings,
    pub(crate) neighborhood_settings: NeighborhoodSettings,
}

impl Default for GridInternalSettings {
    fn default() -> Self {
        GridSettingsBuilder::default().build().0
    }
}

/// [`Point`] represents a single position on the grid.
#[derive(Debug, Default, Clone)]
pub struct Point {
    /// The movement cost associated with this point.
    pub cost: u32,
    /// Solid will block all movement, aka wall.
    pub solid: bool,
    /// Ramp will allow movement up or down.
    pub ramp: bool,
    // Cached neighbors to the point
    pub(crate) neighbor_bits: u32,
}

impl Point {
    pub fn new(cost: u32, solid: bool) -> Self {
        Point {
            cost,
            solid,
            ramp: false,
            neighbor_bits: 0,
        }
    }

    pub fn neighbor_iter(&self, pos: UVec3) -> impl Iterator<Item = UVec3> + '_ {
        let origin = pos.as_ivec3();
        ORDINAL_3D_OFFSETS
            .iter()
            .enumerate()
            .filter_map(move |(i, offset)| {
                if (self.neighbor_bits >> i) & 1 != 0 {
                    Some((origin + *offset).as_uvec3())
                } else {
                    None
                }
            })
    }
}

/// `Grid` is the main `Resource` struct for the crate.
///
/// # Example
/// ```
/// use bevy::prelude::*;
/// use bevy_northstar::prelude::*;
///
/// fn main() {
///     App::new()
///        .add_plugins(DefaultPlugins)
///        .add_plugins(NorthstarPlugin::<CardinalNeighborhood>::default())
///        .add_systems(Startup, startup);
/// }
///
/// fn startup(mut commands: Commands) {
///    // Populate the grid with, you'd usually do this after you load your tilemap
///    // and then set the points in the grid to match the tilemap.
///
///    let mut grid: Grid<CardinalNeighborhood> = Grid::new(&GridSettings {
///       width: 64,
///       height: 64,
///       depth: 1,
///       chunk_size: 16,
///       chunk_depth: 1,
///       chunk_ordinal: false,
///       default_cost: 1,
///       default_wall: false,
///       collision: true,
///       avoidance_distance: 4,
///    });
///
///    grid.set_point(UVec3::new(0, 0, 0), Point::new(1, false));
///    // Initialize the grid
///    grid.build();
///
///    // Add the grid to the world
///    commands.spawn(grid);
/// }
/// ```
#[derive(Component)]
pub struct Grid<N: Neighborhood> {
    pub neighborhood: N,

    dimensions: UVec3,
    chunk_settings: ChunkSettings,
    collision_settings: CollisionSettings,

    grid: Array3<Point>,
    chunks: Array3<Chunk>,

    graph: Graph,
}

impl<N: Neighborhood + Default> Grid<N> {
    /// Creates a new `Grid` instance with the given `GridSettings`.
    pub fn new(settings: &GridSettings) -> Self {
        let GridInternalSettings {
            dimensions,
            chunk_settings,
            cost_settings,
            collision_settings,
            neighborhood_settings: _,
        } = settings.0;

        let UVec3 { x, y, z } = dimensions;

        let x_chunks = x / chunk_settings.size;
        let y_chunks = y / chunk_settings.size;
        let z_chunks = z / chunk_settings.depth;

        let grid = Array3::from_elem(
            (x as usize, y as usize, z as usize),
            Point::new(cost_settings.default_cost, cost_settings.default_solid),
        );

        let chunks = Array3::from_shape_fn(
            (x_chunks as usize, y_chunks as usize, z_chunks as usize),
            |(x, y, z)| {
                let min_x = x as u32 * chunk_settings.size;
                let max_x = min_x + chunk_settings.size - 1;
                let min_y = y as u32 * chunk_settings.size;
                let max_y = min_y + chunk_settings.size - 1;
                let min_z = z as u32 * chunk_settings.depth;
                let max_z = min_z + chunk_settings.depth - 1;

                Chunk::new(
                    UVec3::new(min_x, min_y, min_z),
                    UVec3::new(max_x, max_y, max_z),
                )
            },
        );

        Self {
            neighborhood: N::from_settings(&settings.0.neighborhood_settings),
            dimensions,
            chunk_settings,
            collision_settings,

            grid,
            chunks,

            graph: Graph::new(),
        }
    }

    /// Returns an `ArrayView3` reference to the grid.
    pub fn view(&self) -> ArrayView3<Point> {
        self.grid.view()
    }

    pub(crate) fn chunk_view(&self, chunk: &Chunk) -> ArrayView3<Point> {
        chunk.view(&self.grid)
    }

    pub(crate) fn graph(&self) -> &Graph {
        &self.graph
    }

    /// Returns the `Point` at the given position in the grid.
    pub fn point(&self, pos: UVec3) -> Point {
        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].clone()
    }

    /// Set the `Point` at the given position in the grid.
    pub fn set_point(&mut self, pos: UVec3, point: Point) {
        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]] = point;
    }

    /// Returns the dimensions of the grid.
    pub fn dimensions(&self) -> UVec3 {
        self.dimensions
    }

    /// Returns the width of the grid.
    pub fn width(&self) -> u32 {
        self.dimensions.x
    }

    /// Returns the height of the grid.
    pub fn height(&self) -> u32 {
        self.dimensions.y
    }

    /// Returns the depth of the grid.
    pub fn depth(&self) -> u32 {
        self.dimensions.z
    }

    /// Returns the square width/height of the chunks in the grid.
    pub fn chunk_size(&self) -> u32 {
        self.chunk_settings.size
    }

    /// Returns the chunk settings of the grid.
    pub fn chunk_settings(&self) -> ChunkSettings {
        self.chunk_settings
    }

    /// Returns if collision is enabled or not
    pub fn collision(&self) -> bool {
        self.collision_settings.enabled
    }

    /// Set the collision flag on the graph
    pub fn set_collision(&mut self, collision: bool) {
        self.collision_settings.enabled = collision;
    }

    /// Returns the avoidance distance for collision avoidance
    pub fn avoidance_distance(&self) -> u32 {
        self.collision_settings.avoidance_distance
    }

    /// Set the avoidance distance for collision avoidance
    pub fn set_avoidance_distance(&mut self, distance: u32) {
        self.collision_settings.avoidance_distance = distance;
    }

    /// Builds the entire grid. This includes creating nodes for each edge of each chunk, caching
    /// paths between internal nodes within each chunk, and connecting adjacent nodes between chunks.
    /// This method should be called after the grid has been initialized.
    pub fn build(&mut self) {
        self.precompute_neighbors();
        self.build_nodes();
        self.connect_internal_chunk_nodes();
        self.connect_adjacent_chunk_nodes();
    }

    // Populates the graph with nodes for each edge of each chunk.
    fn build_nodes(&mut self) {
        let chunk_size = self.chunk_settings.size as usize;
        let chunk_depth = self.chunk_settings.depth as usize;

        let x_chunks = self.dimensions.x as usize / chunk_size;
        let y_chunks = self.dimensions.y as usize / chunk_size;
        let z_chunks = self.dimensions.z as usize / chunk_depth;

        for x in 0..x_chunks {
            for y in 0..y_chunks {
                for z in 0..z_chunks {
                    let current_chunk = &self.chunks[[x, y, z]];

                    let directions = Dir::cardinal();

                    for dir in directions {
                        let dir_vec = dir.vector();

                        let nx = x as i32 + dir_vec.0;
                        let ny = y as i32 + dir_vec.1;
                        let nz = z as i32 + dir_vec.2;

                        // Ensure neighbor is within bounds
                        if nx >= 0
                            && nx < x_chunks as i32
                            && ny >= 0
                            && ny < y_chunks as i32
                            && nz >= 0
                            && nz < z_chunks as i32
                        {
                            let neighbor_chunk =
                                &self.chunks[[nx as usize, ny as usize, nz as usize]];

                            let current_edge = current_chunk.edge(&self.grid, dir);
                            let neighbor_edge = neighbor_chunk.edge(&self.grid, dir.opposite());

                            let mut nodes = self.calculate_edge_nodes(
                                current_edge,
                                neighbor_edge,
                                current_chunk.clone(),
                                dir,
                            );

                            // Position the nodes in world space using the fixed axis to realign the nodes,
                            // they also need to be adjusted by the position of the chunk in the grid
                            for node in nodes.iter_mut() {
                                node.pos = match dir {
                                    Dir::NORTH => UVec3::new(
                                        node.pos.x + current_chunk.min().x,
                                        node.pos.y + current_chunk.max().y,
                                        node.pos.z + current_chunk.min().z,
                                    ),
                                    Dir::EAST => UVec3::new(
                                        node.pos.y + current_chunk.max().x,
                                        node.pos.x + current_chunk.min().y,
                                        node.pos.z + current_chunk.min().z,
                                    ),
                                    Dir::SOUTH => UVec3::new(
                                        node.pos.x + current_chunk.min().x,
                                        node.pos.y + current_chunk.min().y,
                                        node.pos.z + current_chunk.min().z,
                                    ),
                                    Dir::WEST => UVec3::new(
                                        node.pos.y + current_chunk.min().x,
                                        node.pos.x + current_chunk.min().y,
                                        node.pos.z + current_chunk.min().z,
                                    ),
                                    // TODO: WE NEED TO FIX UP AND DOWN NODE STUFFS
                                    Dir::UP => UVec3::new(
                                        node.pos.x + current_chunk.min().x,
                                        node.pos.y + current_chunk.min().y,
                                        node.pos.z + current_chunk.max().z,
                                    ),
                                    Dir::DOWN => UVec3::new(
                                        node.pos.x + current_chunk.min().x,
                                        node.pos.y + current_chunk.min().y,
                                        node.pos.z + current_chunk.min().z,
                                    ),
                                    _ => panic!("Invalid direction"),
                                }
                            }

                            self.graph.add_nodes(&nodes);
                        }
                    }

                    // Handle ordinal connections if enabled
                    if self.chunk_settings.diagonal_connections {
                        let ordinal_directions = Dir::ordinal();

                        for dir in ordinal_directions {
                            let dir_vec = dir.vector();

                            let nx = x as i32 + dir_vec.0;
                            let ny = y as i32 + dir_vec.1;
                            let nz = z as i32 + dir_vec.2;

                            // Ensure neighbor is within bounds
                            if nx >= 0
                                && nx < x_chunks as i32
                                && ny >= 0
                                && ny < y_chunks as i32
                                && nz >= 0
                                && nz < z_chunks as i32
                            {
                                let neighbor_chunk =
                                    &self.chunks[[nx as usize, ny as usize, nz as usize]];

                                let current_corner = current_chunk.corner(&self.grid, dir);
                                let neighbor_corner =
                                    neighbor_chunk.corner(&self.grid, dir.opposite());

                                if current_corner.solid || neighbor_corner.solid {
                                    continue;
                                }

                                let pos = match dir {
                                    Dir::NORTHEAST => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.max().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::SOUTHEAST => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.min().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::SOUTHWEST => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.min().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::NORTHWEST => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.max().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::NORTHEASTUP => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.max().y,
                                        current_chunk.max().z,
                                    ),
                                    Dir::SOUTHEASTUP => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.min().y,
                                        current_chunk.max().z,
                                    ),
                                    Dir::SOUTHWESTUP => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.min().y,
                                        current_chunk.max().z,
                                    ),
                                    Dir::NORTHWESTUP => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.max().y,
                                        current_chunk.max().z,
                                    ),
                                    Dir::NORTHEASTDOWN => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.max().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::SOUTHEASTDOWN => UVec3::new(
                                        current_chunk.max().x,
                                        current_chunk.min().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::SOUTHWESTDOWN => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.min().y,
                                        current_chunk.min().z,
                                    ),
                                    Dir::NORTHWESTDOWN => UVec3::new(
                                        current_chunk.min().x,
                                        current_chunk.max().y,
                                        current_chunk.min().z,
                                    ),
                                    _ => panic!("Invalid direction"),
                                };

                                self.graph.add_node(pos, current_chunk.clone(), Some(dir));
                            }
                        }
                    }
                }
            }
        }
    }

    // Calculates the edge nodes for a given edge in the grid.
    fn calculate_edge_nodes(
        &self,
        start_edge: ArrayView2<Point>,
        end_edge: ArrayView2<Point>,
        chunk: Chunk,
        dir: Dir,
    ) -> Vec<Node> {
        let mut nodes = Vec::new();

        // Iterate over the start edge and find connections that are walkable
        for ((start_x, start_y), start_point) in start_edge.indexed_iter() {
            if start_point.solid {
                continue;
            }

            let end_x = start_x;
            let mut end_y = start_y;

            if start_point.ramp {
                end_y = start_y + 1;
            }

            let end_point = end_edge[[end_x, end_y]].clone();

            // If the end point is a wall, we can't connect the nodes
            if !end_point.solid {
                let pos = UVec3::new(start_x as u32, start_y as u32, 0);

                let node = Node::new(pos, chunk.clone(), Some(dir));
                nodes.push(node);
            }
        }

        // Split nodes into groups of continous nodes
        let continous_with = |start: UVec3, end: UVec3| {
            let x_diff = (start.x as i32 - end.x as i32).abs();
            let y_diff = (start.y as i32 - end.y as i32).abs();

            x_diff <= 1 && y_diff <= 1
        };

        let mut split_nodes = Vec::new();
        let mut current_group: Vec<Node> = Vec::new();

        for node in nodes {
            if let Some(last_node) = current_group.last() {
                if !continous_with(last_node.pos, node.pos) {
                    split_nodes.push(current_group);
                    current_group = Vec::new();
                }
            }
            current_group.push(node);
        }

        if !current_group.is_empty() {
            split_nodes.push(current_group);
        }

        // Find the center of the split nodes
        let mut final_nodes = Vec::new();

        for group in &split_nodes {
            /*if group.len() > 8 {
                final_nodes.push(group.first().unwrap().clone());
                final_nodes.push(group.last().unwrap().clone());
            } else {*/

            let middle = group.len() / 2;
            final_nodes.push(group[middle].clone());
            //}
        }

        if !final_nodes.is_empty() {
            return final_nodes;
        }

        if !self.neighborhood.is_ordinal() {
            return final_nodes;
        }

        let mut ordinal_nodes = Vec::new();

        // If we made it here that means no connection nodes were found, but we allow ordinal connections so let's build those if possible
        for ((start_x, start_y), start_point) in start_edge.indexed_iter() {
            if start_point.solid {
                continue;
            }

            let end_x = start_x;
            let mut end_y = start_y;

            if start_point.ramp {
                end_y = start_y + 1;
            }

            if end_x > 0 {
                let left = end_edge[[end_x - 1, end_y]].clone();
                if !left.solid {
                    let pos = UVec3::new(start_x as u32, start_y as u32, 0);

                    let node = Node::new(pos, chunk.clone(), Some(dir));
                    ordinal_nodes.push(node);
                }
            }

            if end_x < end_edge.shape()[0] - 1 {
                let right = end_edge[[end_x + 1, end_y]].clone();
                if !right.solid {
                    let pos = UVec3::new(start_x as u32, start_y as u32, 0);

                    let node = Node::new(pos, chunk.clone(), Some(dir));
                    ordinal_nodes.push(node);
                }
            }
        }

        ordinal_nodes
    }

    // Connects the internal nodes of each chunk to each other.
    fn connect_internal_chunk_nodes(&mut self) {
        let chunk_size = self.chunk_settings.size as usize;
        let chunk_depth = self.chunk_settings.depth as usize;

        let x_chunks = self.dimensions.x as usize / chunk_size;
        let y_chunks = self.dimensions.y as usize / chunk_size;
        let z_chunks = self.dimensions.z as usize / chunk_depth;

        for x in 0..x_chunks {
            for y in 0..y_chunks {
                for z in 0..z_chunks {
                    // Connect internal nodes

                    let chunk_grid = self.chunks[[x, y, z]].view(&self.grid);
                    let chunk = &self.chunks[[x, y, z]];

                    let nodes = self.graph.nodes_in_chunk(chunk.clone());

                    let mut connections = Vec::new();

                    for node in nodes.iter() {
                        // Collect other nodes positions into an array
                        let other_nodes = nodes
                            .iter()
                            .filter(|other_node| other_node.pos != node.pos)
                            .map(|other_node| other_node.pos)
                            .collect::<Vec<_>>();

                        // Adjust node.pos by the chunk position
                        let start_pos = node.pos - chunk.min();

                        let goals = other_nodes
                            .iter()
                            .map(|pos| *pos - chunk.min())
                            .collect::<Vec<_>>();

                        let paths = dijkstra_grid(
                            &chunk_grid,
                            start_pos,
                            &goals,
                            false,
                            100,
                            &HashMap::new(),
                        );

                        // Readjust position to world space and then connect the nodes
                        for (goal_pos, path) in paths {
                            let start = node.pos;
                            let goal = goal_pos + chunk.min();

                            // Readjust path to world space
                            let path_vec = path
                                .path()
                                .iter()
                                .map(|pos| *pos + chunk.min())
                                .collect::<Vec<_>>();

                            connections.push((
                                start,
                                goal,
                                Path::new(path_vec.clone(), path_vec.len() as u32),
                            ));
                        }
                    }

                    for (node_pos, other_node_pos, path) in connections {
                        self.graph.connect_node(node_pos, other_node_pos, path);
                    }
                }
            }
        }
    }

    // Connects the nodes of adjacent chunks to each other.
    fn connect_adjacent_chunk_nodes(&mut self) {
        let nodes = self.graph.nodes();

        let mut connections = Vec::new();

        for node in nodes {
            // Check all the adjacent positions of the node, taking into account cardinal/ordinal settings
            let directions = if self.chunk_settings.diagonal_connections {
                Dir::all()
            } else {
                Dir::cardinal()
            };

            for dir in directions {
                let dir_vec = dir.vector();

                let nx = node.pos.x as i32 + dir_vec.0;
                let ny = node.pos.y as i32 + dir_vec.1;
                let nz = node.pos.z as i32 + dir_vec.2;

                if let Some(neighbor) = self
                    .graph
                    .node_at(UVec3::new(nx as u32, ny as u32, nz as u32))
                {
                    // Check if neighbor is in a different chunk
                    if node.chunk != neighbor.chunk {
                        let path = Path::from_slice(&[node.pos, neighbor.pos], 1);

                        connections.push((node.pos, neighbor.pos, path));
                    }
                }
            }
        }

        for (node_pos, neighbor_pos, path) in connections {
            self.graph.connect_node(node_pos, neighbor_pos, path);
        }
    }

    /*pub(crate) fn precompute_neighbors(&mut self) {
        // Collect all positions first to avoid borrow checker issues
        let positions: Vec<_> = self.grid.indexed_iter().map(|(pos, _)| pos).collect();
        let grid_view = self.grid.view();

        // Compute the bitfield for each point
        let mut bitfields = Vec::with_capacity(positions.len());
        for pos in &positions {
            let neighbor_bits = self.neighborhood.neighbors(
                &grid_view,
                UVec3::new(pos.0 as u32, pos.1 as u32, pos.2 as u32),
            );
            bitfields.push(neighbor_bits);
        }

        // Apply the bitfield to the grid
        for (i, pos) in positions.iter().enumerate() {
            self.grid[[pos.0, pos.1, pos.2]].neighbor_bits = bitfields[i];
        }
    }*/

    pub(crate) fn precompute_neighbors(&mut self) {
        let positions: Vec<_> = self.grid.indexed_iter().map(|(pos, _)| pos).collect();
        let shape = self.grid.dim(); // (x, y, z)
        let grid_view = self.grid.view();

        // First, collect all neighbor_bits for each position
        let mut bitfields = Vec::with_capacity(positions.len());
        for pos in &positions {
            let uvec_pos = UVec3::new(pos.0 as u32, pos.1 as u32, pos.2 as u32);
            let mut neighbor_bits = 0;

            for (i, offset) in ORDINAL_3D_OFFSETS.iter().enumerate() {
                let neighbor = uvec_pos.as_ivec3() + *offset;

                // Bounds check — skip invalid neighbors
                if neighbor.cmplt(IVec3::ZERO).any() {
                    continue;
                }
                if neighbor.x >= shape.0 as i32 || neighbor.y >= shape.1 as i32 || neighbor.z >= shape.2 as i32 {
                    continue;
                }

                // Add to bitmask only if the neighbor is not solid
                let neighbor_uvec = UVec3::new(neighbor.x as u32, neighbor.y as u32, neighbor.z as u32);
                let neighbor_point = &grid_view[[neighbor_uvec.x as usize, neighbor_uvec.y as usize, neighbor_uvec.z as usize]];

                if !neighbor_point.solid {
                    neighbor_bits |= 1 << i;
                }
            }

            bitfields.push(neighbor_bits);
        }

        // Now, apply the neighbor_bits to the grid
        for (i, pos) in positions.iter().enumerate() {
            self.grid[[pos.0, pos.1, pos.2]].neighbor_bits = bitfields[i];
        }
    }


    /// Returns the `Chunk` for the given position `UVec3` in the grid.
    pub(crate) fn chunk_at_position(&self, pos: UVec3) -> Option<&Chunk> {
        self.chunks.iter().find(|&chunk| {
            pos.x >= chunk.min().x
                && pos.x <= chunk.max().x
                && pos.y >= chunk.min().y
                && pos.y <= chunk.max().y
                && pos.z >= chunk.min().z
                && pos.z <= chunk.max().z
        })
    }

    /// Recursively reroutes a path by astar pathing to further chunks until a path can be found.
    ///
    /// Useful if local collision avoidance is failing.
    ///
    /// If you're using the plugin pathing systems, you shouldn't need to call this directly.
    ///
    /// # Arguments
    pub fn reroute_path(
        &self,
        path: &Path,
        start: UVec3,
        goal: UVec3,
        blocking: &HashMap<UVec3, Entity>,
    ) -> Option<Path> {
        reroute_path(self, path, start, goal, blocking)
    }

    pub fn pathfind(
        &self,
        start: UVec3,
        goal: UVec3,
        blocking: &HashMap<UVec3, Entity>,
        partial: bool,
    ) -> Option<Path> {
        pathfind(self, start, goal, blocking, partial)
    }

    pub fn pathfind_astar(
        &self,
        start: UVec3,
        goal: UVec3,
        blocking: &HashMap<UVec3, Entity>,
        partial: bool,
    ) -> Option<Path> {
        pathfind_astar(
            &self.neighborhood,
            &self.grid.view(),
            start,
            goal,
            blocking,
            partial,
        )
    }
}

#[cfg(test)]
mod tests {
    use bevy::{math::UVec3, platform::collections::HashMap};

    use crate::{
        dir::Dir,
        grid::{
            ChunkSettings, CollisionSettings, CostSettings, Grid, GridInternalSettings,
            GridSettings, GridSettingsBuilder, NeighborhoodSettings, Point,
        },
        neighbor::OrdinalNeighborhood3d,
    };

    const GRID_SETTINGS: GridSettings = GridSettings(GridInternalSettings {
        dimensions: UVec3::new(12, 12, 1),
        chunk_settings: ChunkSettings {
            size: 4,
            depth: 1,
            diagonal_connections: false,
        },
        cost_settings: CostSettings {
            default_cost: 1,
            default_solid: false,
        },
        collision_settings: CollisionSettings {
            enabled: false,
            avoidance_distance: 4,
        },
        neighborhood_settings: NeighborhoodSettings {
            allow_corner_clipping: true,
            smooth_corner_paths: false,
        },
    });

    #[test]
    pub fn test_new() {
        let grid = Grid::<OrdinalNeighborhood3d>::new(&GRID_SETTINGS);
        assert_eq!(grid.grid.shape(), [12, 12, 1]);
    }

    #[test]
    pub fn test_edges() {
        let grid_settings = GridSettingsBuilder::new_2d(4, 4)
            .chunk_size(4)
            .enable_diagonal_connections()
            .build();

        let mut grid = Grid::<OrdinalNeighborhood3d>::new(&grid_settings);

        // Fill grid edges with walls
        for x in 0..4 {
            for y in 0..4 {
                if x == 0 || x == 3 || y == 0 || y == 3 {
                    grid.grid[[x, y, 0]] = Point::new(1, true);
                }
            }
        }

        let chunk = grid.chunks[[0, 0, 0]].clone();

        let mut edges = Vec::new();

        edges.push(chunk.edge(&grid.grid, Dir::NORTH));
        edges.push(chunk.edge(&grid.grid, Dir::EAST));
        edges.push(chunk.edge(&grid.grid, Dir::SOUTH));
        edges.push(chunk.edge(&grid.grid, Dir::WEST));

        for edge in edges {
            for point in edge.iter() {
                assert!(point.solid);
            }
        }
    }

    #[test]
    pub fn test_calculate_edge_nodes() {
        let grid = Grid::<OrdinalNeighborhood3d>::new(&GRID_SETTINGS);

        let chunk = grid.chunks.iter().next().unwrap().clone();
        let neighbor_chunk = grid.chunks.iter().nth(1).unwrap().clone();

        let edges = grid.calculate_edge_nodes(
            chunk.edge(&grid.grid, Dir::NORTH),
            neighbor_chunk.edge(&grid.grid, Dir::SOUTH),
            chunk.clone(),
            Dir::NORTH,
        );

        assert_eq!(edges.len(), 1);
    }

    #[test]
    pub fn test_calculate_edge_nodes_3d() {
        let grid_settings = GridSettingsBuilder::new_3d(8, 8, 8)
            .chunk_size(4)
            .chunk_depth(4)
            .enable_diagonal_connections()
            .build();

        let grid = Grid::<OrdinalNeighborhood3d>::new(&grid_settings);

        let chunk = grid.chunks.iter().next().unwrap().clone();
        let neighbor_chunk = grid.chunks.iter().nth(1).unwrap().clone();

        let edges = grid.calculate_edge_nodes(
            chunk.edge(&grid.grid, Dir::NORTH),
            neighbor_chunk.edge(&grid.grid, Dir::SOUTH),
            chunk.clone(),
            Dir::NORTH,
        );

        assert_eq!(edges.len(), 4);
    }

    #[test]
    pub fn test_build_nodes() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build_nodes();

        let nodes = grid.graph.nodes();

        assert_eq!(nodes[0].pos, UVec3::new(2, 3, 0));
        assert_eq!(nodes[1].pos, UVec3::new(3, 2, 0));

        assert_eq!(nodes[2].pos, UVec3::new(2, 7, 0));
        assert_eq!(nodes[3].pos, UVec3::new(3, 6, 0));
        assert_eq!(nodes[4].pos, UVec3::new(2, 4, 0));

        assert_eq!(nodes[5].pos, UVec3::new(3, 10, 0));
        assert_eq!(nodes[6].pos, UVec3::new(2, 8, 0));

        assert_eq!(grid.graph.node_count(), 24);

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        let chunk_size = GRID_SETTINGS.0.chunk_settings.size as usize;
        let half_chunk_size = chunk_size / 2;

        for x in 0..(GRID_SETTINGS.0.dimensions.x as usize) {
            for y in 0..(GRID_SETTINGS.0.dimensions.y as usize) {
                if x % chunk_size == 0 && y % half_chunk_size == 0 {
                    grid.grid[[x, y, 0]] = Point::new(0, true);
                } else {
                    grid.grid[[x, y, 0]] = Point::new(0, false);
                }
            }
        }

        grid.build_nodes();

        assert_eq!(grid.graph.node_count(), 36);

        let grid_settings = GridSettingsBuilder::new_2d(12, 12)
            .chunk_size(4)
            .enable_diagonal_connections()
            .build();

        // Test ordinal
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        grid.build_nodes();

        assert_eq!(grid.graph.node_count(), 40);
    }

    #[test]
    pub fn test_connect_internal_nodes() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.precompute_neighbors();
        grid.build_nodes();
        grid.connect_internal_chunk_nodes();

        assert_eq!(grid.graph.node_count(), 24);
        assert_eq!(grid.graph.edge_count(), 44);
    }

    #[test]
    pub fn test_connect_adjacent_nodes() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.precompute_neighbors();
        grid.build_nodes();
        grid.connect_adjacent_chunk_nodes();

        let edges = grid
            .graph
            .node_at(UVec3::new(2, 3, 0))
            .unwrap()
            .edges
            .clone();

        assert!(edges.contains_key(&UVec3::new(2, 4, 0)));

        assert_eq!(edges[&UVec3::new(2, 4, 0)].path().len(), 2);
        assert_eq!(edges[&UVec3::new(2, 4, 0)].cost(), 1);

        assert_eq!(grid.graph.edge_count(), 24);
    }

    #[test]
    pub fn test_get_chunk_for_position() {
        let grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);
        let chunk = grid.chunk_at_position(UVec3::new(0, 0, 0)).unwrap();

        assert_eq!(chunk.min(), UVec3::new(0, 0, 0));
        assert_eq!(
            chunk.max(),
            UVec3::new(
                GRID_SETTINGS.0.chunk_settings.size - 1,
                GRID_SETTINGS.0.chunk_settings.size - 1,
                GRID_SETTINGS.0.chunk_settings.depth - 1
            )
        );
    }

    #[test]
    pub fn test_get_all_nodes_in_chunk() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);
        grid.build_nodes();

        let nodes = grid.graph.nodes_in_chunk(grid.chunks[[0, 0, 0]].clone());

        assert_eq!(nodes.len(), 2);
    }

    #[test]
    pub fn test_get_path() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = grid.pathfind(
            UVec3::new(10, 10, 0),
            UVec3::new(4, 4, 0),
            &HashMap::new(),
            false,
        );
        let raw_path = grid.pathfind_astar(
            UVec3::new(10, 10, 0),
            UVec3::new(4, 4, 0),
            &HashMap::new(),
            false,
        );

        assert!(path.is_some());
        // Ensure start point is the first point in the path
        assert_ne!(path.clone().unwrap().path()[0], UVec3::new(10, 10, 0));
        // Ensure end point is the last point in the path
        assert_eq!(
            path.clone().unwrap().path().last().unwrap(),
            &UVec3::new(4, 4, 0)
        );

        assert_eq!(path.clone().unwrap().len(), raw_path.unwrap().len());
        assert_eq!(path.unwrap().len(), 6);
    }

    #[test]
    fn test_calculate_edge_nodes_returns_center() {
        let grid_settings = GridSettingsBuilder::new_2d(64, 64).chunk_size(32).build();

        let grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        let start_edge = grid.chunks[[0, 0, 0]].edge(&grid.grid, Dir::NORTH);
        let end_edge = grid.chunks[[0, 1, 0]].edge(&grid.grid, Dir::SOUTH);

        let nodes = grid.calculate_edge_nodes(
            start_edge,
            end_edge,
            grid.chunks[[0, 0, 0]].clone(),
            Dir::NORTH,
        );

        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].pos, UVec3::new(16, 0, 0));
    }

    #[test]
    fn test_random_grid_path() {
        let width = 128;
        let height = 128;
        let chunk_size = 32;

        // Test a grid with randomized walls, the grid must be solvable
        let grid_settings = GridSettingsBuilder::new_2d(width, height)
            .chunk_size(chunk_size)
            .enable_diagonal_connections()
            .build();

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        for x in 0..width {
            for y in 0..height {
                if (x % 4 == 0)
                    && (y % chunk_size == 0 || y / (chunk_size + chunk_size) == chunk_size)
                {
                    grid.grid[[x as usize, y as usize, 0]] = Point::new(1, true);
                }

                if (y % 4 == 0)
                    && (x % chunk_size == 0 || x & (chunk_size + chunk_size) == chunk_size)
                {
                    grid.grid[[x as usize, y as usize, 0]] = Point::new(1, true);
                }
            }
        }

        grid.build();

        let path = grid.pathfind(
            UVec3::new(7, 7, 0),
            UVec3::new(121, 121, 0),
            &HashMap::new(),
            false,
        );

        assert!(path.is_some());
        assert!(!path.unwrap().is_empty());
    }

    #[test]
    fn test_large_3d_path() {
        let grid_settings = GridSettingsBuilder::new_3d(128, 128, 4)
            .chunk_size(16)
            .chunk_depth(2)
            .build();

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        grid.build();
        let path = grid.pathfind(
            UVec3::new(0, 0, 0),
            UVec3::new(31, 31, 3),
            &HashMap::new(),
            false,
        );

        assert!(path.is_some());
    }

    #[test]
    pub fn test_get_astar_path() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = grid.pathfind_astar(
            UVec3::new(0, 0, 0),
            UVec3::new(10, 10, 0),
            &HashMap::new(),
            false,
        );

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 10);
    }

    #[test]
    fn test_neighbor_iter_on_grid_edge() {
        // Set up a dummy Point with all 26 neighbor bits enabled
        let mut point = Point::default();
        point.neighbor_bits = u32::MAX >> (32 - 26); // Only lower 26 bits set

        // Use an edge position (e.g., corner of the grid)
        let pos = UVec3::new(0, 0, 0);

        // Collect neighbors
        let neighbors: Vec<UVec3> = point.neighbor_iter(pos).collect();

        // Should only include neighbors that are within bounds
        // From (0,0,0), valid neighbors are ones with only positive offsets:
        // i.e., (1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 0, 1), (0, 1, 1), (1, 1, 1)
        let expected = [
            UVec3::new(1, 0, 0),
            UVec3::new(0, 1, 0),
            UVec3::new(0, 0, 1),
            UVec3::new(1, 1, 0),
            UVec3::new(1, 0, 1),
            UVec3::new(0, 1, 1),
            UVec3::new(1, 1, 1),
        ];

        assert_eq!(neighbors.len(), expected.len());
        for expected_neighbor in expected {
            assert!(
                neighbors.contains(&expected_neighbor),
                "Missing expected neighbor: {:?}",
                expected_neighbor
            );
        }
    }

    #[test]
    fn test_neighbor_iter_bounds_check() {
        let mut point = Point::default();
        point.neighbor_bits = u32::MAX; // Enable all 26 bits

        let pos = UVec3::new(0, 0, 0);
        let neighbors: Vec<_> = point.neighbor_iter(pos).collect();

        // Only 7 neighbors should remain valid at (0,0,0)
        assert_eq!(neighbors.len(), 7);

        // Ensure no neighbor has any coordinate == u32::MAX (from underflow)
        for neighbor in neighbors {
            assert!(
                neighbor.x < pos.x + 2 && neighbor.y < pos.y + 2 && neighbor.z < pos.z + 2,
                "Invalid neighbor: {:?}",
                neighbor
            );
        }
    }
}
