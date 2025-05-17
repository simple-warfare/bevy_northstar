//! This module contains the `Grid` struct which is the primary `Resource` for the crate.
use bevy::{
    math::UVec3,
    prelude::{Entity, Component},
    platform::collections::HashMap,
};
use ndarray::{Array3, ArrayView2, ArrayView3};

use crate::{
    chunk::Chunk,
    dijkstra::*,
    dir::*,
    graph::Graph,
    neighbor::Neighborhood,
    node::Node,
    path::Path,
    pathfind::{pathfind, pathfind_astar, reroute_path},
};

/// Settings for configuring the grid.
pub struct GridSettings {
    /// The width of the grid.
    pub width: u32,
    /// The height of the grid.
    pub height: u32,
    /// The depth of the grid. Use 1 for 2D grids.
    pub depth: u32,

    /// The size of each chunk the grid should be divided into for HPA*.
    pub chunk_size: u32,
    /// The depth of each chunk the grid should be divided into for HPA*.
    pub chunk_depth: u32,

    /// Create entrances to chunks in corners.
    pub chunk_ordinal: bool,

    /// The default cost associated with grid positions.
    pub default_cost: u32,
    /// Set to true to default all grid positions to walls.
    pub default_wall: bool,

    /// Collision
    pub collision: bool,
    
    /// Collision Avoidance distance
    pub avoidance_distance: u32,
}

impl Default for GridSettings {
    fn default() -> Self {
        GridSettings {
            width: 64,
            height: 64,
            depth: 1,
            chunk_size: 16,
            chunk_depth: 1,
            chunk_ordinal: false,
            default_cost: 1,
            default_wall: false,
            collision: false,
            avoidance_distance: 4,
        }
    }
}

/// [`Point`] represents a single position on the grid.
#[derive(Debug, Default, Clone, Copy)]
pub struct Point {
    /// The movement cost associated with this point.
    pub cost: u32,
    /// Wall will block all movement.
    pub wall: bool,
    /// Ramp will allow movement up or down.
    pub ramp: bool,
}

impl Point {
    pub fn new(cost: u32, wall: bool) -> Self {
        Point {
            cost,
            wall,
            ramp: false,
        }
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
///        .insert_resource(Grid::<CardinalNeighborhood>::new(&GridSettings {
///            width: 128,
///            height: 128,
///            depth: 1,
///            chunk_size: 16,
///            chunk_depth: 1,
///            chunk_ordinal: true,
///            default_cost: 1,
///            default_wall: false,
///            collision: false,
///            avoidance_distance: 4,
///        }))
///        .add_systems(Startup, startup);
/// }
///
/// fn startup(mut commands: Commands, mut grid: ResMut<Grid<CardinalNeighborhood>>) {
///    // Populate the grid with, you'd usually do this after you load your tilemap
///    // and then set the points in the grid to match the tilemap.
///    grid.set_point(UVec3::new(0, 0, 0), Point::new(1, false));
///    // Initialize the grid
///    grid.build();
/// }
/// ```
#[derive(Component)]
pub struct Grid<N: Neighborhood> {
    pub neighborhood: N,

    width: u32,
    height: u32,
    depth: u32,

    chunk_size: u32,
    chunk_depth: u32,

    chunk_ordinal: bool,

    #[allow(dead_code)]
    default_cost: u32,
    #[allow(dead_code)]
    default_wall: bool,

    collision: bool,
    avoidance_distance: u32,

    grid: Array3<Point>,
    chunks: Array3<Chunk>,

    graph: Graph,
}

impl<N: Neighborhood + Default> Grid<N> {
    /// Creates a new `Grid` instance with the given `GridSettings`.
    pub fn new(settings: &GridSettings) -> Self {
        let x_chunks = settings.width as usize / settings.chunk_size as usize;
        let y_chunks = settings.height as usize / settings.chunk_size as usize;
        let z_chunks = settings.depth as usize / settings.chunk_depth as usize;

        Grid {
            neighborhood: N::default(),
            width: settings.width,
            height: settings.height,
            depth: settings.depth,
            chunk_size: settings.chunk_size,
            chunk_depth: settings.chunk_depth,
            chunk_ordinal: settings.chunk_ordinal,
            default_cost: settings.default_cost,
            default_wall: settings.default_wall,
            collision: settings.collision,
            avoidance_distance: settings.avoidance_distance,
            grid: Array3::from_elem(
                (
                    settings.width as usize,
                    settings.height as usize,
                    settings.depth as usize,
                ),
                Point::new(settings.default_cost, settings.default_wall),
            ),
            chunks: Array3::from_shape_fn((x_chunks, y_chunks, z_chunks), |(x, y, z)| {
                let min_x = x as u32 * settings.chunk_size;
                let max_x = min_x + settings.chunk_size - 1;
                let min_y = y as u32 * settings.chunk_size;
                let max_y = min_y + settings.chunk_size - 1;
                let min_z = z as u32 * settings.chunk_depth;
                let max_z = min_z + settings.chunk_depth - 1;

                Chunk::new(
                    UVec3::new(min_x, min_y, min_z),
                    UVec3::new(max_x, max_y, max_z),
                )
            }),
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
        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]]
    }

    /// Set the `Point` at the given position in the grid.
    pub fn set_point(&mut self, pos: UVec3, point: Point) {
        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]] = point;
    }

    /// Returns the width of the grid.
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Returns the height of the grid.
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Returns the depth of the grid.
    pub fn depth(&self) -> u32 {
        self.depth
    }

    /// Returns the size of each chunk in the grid.
    pub fn chunk_size(&self) -> u32 {
        self.chunk_size
    }

    /// Returns if collision is enabled on the graph
    pub fn collision(&self) -> bool {
        self.collision
    }

    /// Set the collision flag on the graph
    pub fn set_collision(&mut self, collision: bool) {
        self.collision = collision;
    }

    /// Returns the avoidance distance for collision avoidance
    pub fn avoidance_distance(&self) -> u32 {
        self.avoidance_distance
    }

    /// Set the avoidance distance for collision avoidance
    pub fn set_avoidance_distance(&mut self, distance: u32) {
        self.avoidance_distance = distance;
    }

    /// Builds the entire grid. This includes creating nodes for each edge of each chunk, caching
    /// paths between internal nodes within each chunk, and connecting adjacent nodes between chunks.
    /// This method should be called after the grid has been initialized.
    pub fn build(&mut self) {
        self.build_nodes();
        self.connect_internal_chunk_nodes();
        self.connect_adjacent_chunk_nodes();
    }

    // Populates the graph with nodes for each edge of each chunk.
    fn build_nodes(&mut self) {
        let chunk_size = self.chunk_size as usize;
        let chunk_depth = self.chunk_depth as usize;

        let x_chunks = self.width as usize / chunk_size;
        let y_chunks = self.height as usize / chunk_size;
        let z_chunks = self.depth as usize / chunk_depth;

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
                    if self.chunk_ordinal {
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

                                if current_corner.wall || neighbor_corner.wall {
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
            if start_point.wall {
                continue;
            }

            let end_x = start_x;
            let mut end_y = start_y;

            if start_point.ramp {
                end_y = start_y + 1;
            }

            let end_point = end_edge[[end_x, end_y]];

            // If the end point is a wall, we can't connect the nodes
            if !end_point.wall {
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
            if start_point.wall {
                continue;
            }

            let end_x = start_x;
            let mut end_y = start_y;

            if start_point.ramp {
                end_y = start_y + 1;
            }

            if end_x > 0 {
                let left = end_edge[[end_x - 1, end_y]];
                if !left.wall {
                    let pos = UVec3::new(start_x as u32, start_y as u32, 0);

                    let node = Node::new(pos, chunk.clone(), Some(dir));
                    ordinal_nodes.push(node);
                }
            }

            if end_x < end_edge.shape()[0] - 1 {
                let right = end_edge[[end_x + 1, end_y]];
                if !right.wall {
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
        let chunk_size = self.chunk_size as usize;
        let chunk_depth = self.chunk_depth as usize;

        let x_chunks = self.width as usize / chunk_size;
        let y_chunks = self.height as usize / chunk_size;
        let z_chunks = self.depth as usize / chunk_depth;

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
                            &self.neighborhood,
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
            let directions = if self.chunk_ordinal {
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

    /// Returns the `Chunk` for the given position `UVec3` in the grid.
    pub(crate) fn chunk_at_position(&self, pos: UVec3) -> Option<&Chunk> {
        for chunk in self.chunks.iter() {
            if pos.x >= chunk.min().x
                && pos.x <= chunk.max().x
                && pos.y >= chunk.min().y
                && pos.y <= chunk.max().y
                && pos.z >= chunk.min().z
                && pos.z <= chunk.max().z
            {
                return Some(chunk);
            }
        }

        None
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
        reroute_path(
            &self,
            path,
            start,
            goal,
            blocking,
        )
    }

    pub fn pathfind(
        &self,
        start: UVec3,
        goal: UVec3,
        blocking: &HashMap<UVec3, Entity>,
        partial: bool,
    ) -> Option<Path> {
        pathfind(&self, start, goal, blocking, partial)
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
        grid::{Grid, GridSettings, Point},
        neighbor::OrdinalNeighborhood3d,
    };

    const GRID_SETTINGS: GridSettings = GridSettings {
        width: 12,
        height: 12,
        depth: 1,
        chunk_size: 4,
        chunk_depth: 1,
        chunk_ordinal: false,
        default_cost: 1,
        default_wall: false,
        collision: false,
        avoidance_distance: 4,
    };

    #[test]
    pub fn test_new() {
        let grid = Grid::<OrdinalNeighborhood3d>::new(&GRID_SETTINGS);
        assert_eq!(grid.grid.shape(), [12, 12, 1]);
    }

    #[test]
    pub fn test_edges() {
        let mut grid = Grid::<OrdinalNeighborhood3d>::new(&GridSettings {
            width: 4,
            height: 4,
            depth: 1,
            chunk_size: 4,
            chunk_depth: 1,
            chunk_ordinal: true,
            default_cost: 1,
            default_wall: false,
            collision: false,
            avoidance_distance: 4,
        });

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
                assert_eq!(point.wall, true);
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
        let grid = Grid::<OrdinalNeighborhood3d>::new(&GridSettings {
            width: 8,
            height: 8,
            depth: 8,
            chunk_size: 4,
            chunk_depth: 4,
            chunk_ordinal: true,
            default_cost: 1,
            default_wall: false,
            collision: false,
            avoidance_distance: 4,
        });

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

        let chunk_size = GRID_SETTINGS.chunk_size as usize;
        let half_chunk_size = chunk_size / 2;

        for x in 0..(GRID_SETTINGS.width as usize) {
            for y in 0..(GRID_SETTINGS.height as usize) {
                if x % chunk_size == 0 && y % half_chunk_size == 0 {
                    grid.grid[[x, y, 0]] = Point::new(0, true);
                } else if y % chunk_size == 0 && x % half_chunk_size == 0 {
                    grid.grid[[x, y, 0]] = Point::new(0, true);
                } else {
                    grid.grid[[x, y, 0]] = Point::new(0, false);
                }
            }
        }

        grid.build_nodes();

        assert_eq!(grid.graph.node_count(), 48);

        // Test ordinal
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GridSettings {
            width: 12,
            height: 12,
            depth: 1,
            chunk_size: 4,
            chunk_depth: 1,
            chunk_ordinal: true,
            default_cost: 1,
            default_wall: false,
            collision: false,
            avoidance_distance: 4,
        });

        grid.build_nodes();

        assert_eq!(grid.graph.node_count(), 40);
    }

    #[test]
    pub fn test_connect_internal_nodes() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build_nodes();
        grid.connect_internal_chunk_nodes();

        assert_eq!(grid.graph.node_count(), 24);
        assert_eq!(grid.graph.edge_count(), 44);
    }

    #[test]
    pub fn test_connect_adjacent_nodes() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build_nodes();
        grid.connect_adjacent_chunk_nodes();

        let edges = grid
            .graph
            .node_at(UVec3::new(2, 3, 0))
            .unwrap()
            .edges
            .clone();

        assert_eq!(edges.contains_key(&UVec3::new(2, 4, 0)), true);

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
                GRID_SETTINGS.chunk_size - 1,
                GRID_SETTINGS.chunk_size - 1,
                GRID_SETTINGS.chunk_depth - 1
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
        let grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GridSettings {
            width: 64,
            height: 64,
            depth: 1,
            chunk_size: 32,
            chunk_depth: 1,
            chunk_ordinal: false,
            default_cost: 0,
            default_wall: false,
            collision: false,
            avoidance_distance: 4,
        });

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
        // Test a grid with randomized walls, the grid must be solvable
        let width = 128;
        let height = 128;
        let depth = 1;
        let chunk_size = 32;
        let chunk_depth = 1;

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GridSettings {
            width,
            height,
            depth,
            chunk_size,
            chunk_depth,
            chunk_ordinal: true,
            default_cost: 1,
            default_wall: false,
            collision: false,
            avoidance_distance: 4,
        });

        for x in 0..width {
            for y in 0..height {
                if (x % 4 == 0)
                    && (y % chunk_size == 0 || y / chunk_size + chunk_size == chunk_size)
                {
                    grid.grid[[x as usize, y as usize, 0]] = Point::new(1, true);
                }

                if (y % 4 == 0)
                    && (x % chunk_size == 0 || x & chunk_size + chunk_size == chunk_size)
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
        assert!(path.unwrap().len() > 0);
    }

    #[test]
    fn test_large_3d_path() {
        let grid_settings = GridSettings {
            width: 128,
            height: 128,
            depth: 4,
            chunk_depth: 1,
            chunk_size: 16,
            chunk_ordinal: false,
            default_cost: 1,
            default_wall: false,
            collision: false,
            avoidance_distance: 4,
        };

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
}
