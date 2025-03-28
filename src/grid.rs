use bevy::{math::UVec3, prelude::{Entity, Resource}, utils::hashbrown::HashMap};
use ndarray::{Array3, ArrayView2, ArrayView3};

use crate::{
    astar::{astar_graph, astar_grid}, chunk::Chunk, dijkstra::dijkstra_grid, dir::{get_movement_type, Dir}, graph::Graph, neighbor::Neighborhood, node::Node, path::Path, raycast::{generate_path_segment_cardinal, generate_path_segment_ordinal, line_of_sight_cardinal, line_of_sight_ordinal}, Point
};

/// Settings for configuring the grid.
///
/// # Fields
///
/// * `width` - The width of the grid.
/// * `height` - The height of the grid.
/// * `depth` - The depth of the grid. Use 1 for 2D grids.
///
/// * `chunk_size` - The size of each chunk in the grid.
/// * `chunk_depth` - The depth of each chunk in the grid. Use 1 for 2D grids.
///
/// * `chunk_ordinal` - Create entrances to chunks in corners.
///
/// * `default_cost` - The default cost associated with grid cells.
/// * `default_wall` - A boolean indicating if the default grid cell is a wall.
pub struct GridSettings {
    pub width: u32,
    pub height: u32,
    pub depth: u32,

    pub chunk_size: u32,
    pub chunk_depth: u32,

    pub chunk_ordinal: bool,

    pub default_cost: u32,
    pub default_wall: bool,
}

#[derive(Resource)]
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

    grid: Array3<Point>,
    pub chunks: Array3<Chunk>,

    pub graph: Graph,
}

impl<N: Neighborhood + Default> Grid<N> {
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
    pub fn get_view(&self) -> ArrayView3<Point> {
        self.grid.view()
    }

    /// Set the `Point` at the given position in the grid.
    pub fn set_point(&mut self, pos: UVec3, point: Point) {
        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]] = point;
    }

    /// Get the `Point` at the given position in the grid.
    pub fn get_point(&self, pos: UVec3) -> Point {
        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]]
    }

    /// Returns the width of the grid.
    pub fn get_width(&self) -> u32 {
        self.width
    }

    /// Returns the height of the grid.
    pub fn get_height(&self) -> u32 {
        self.height
    }

    /// Returns the depth of the grid.
    pub fn get_depth(&self) -> u32 {
        self.depth
    }

    /// Returns the size of each chunk in the grid.
    pub fn get_chunk_size(&self) -> u32 {
        self.chunk_size
    }

    /// Builds the entire grid. This includes creating nodes for each edge of each chunk, caching
    /// paths between internal nodes within each chunk, and connecting adjacent nodes between chunks.
    /// This method should be called after the grid has been initialized.
    pub fn build(&mut self) {
        self.build_nodes();
        self.connect_internal_chunk_nodes();
        self.connect_adjacent_chunk_nodes();
    }

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
                                        node.pos.x + current_chunk.min.x,
                                        node.pos.y + current_chunk.max.y,
                                        node.pos.z + current_chunk.min.z,
                                    ),
                                    Dir::EAST => UVec3::new(
                                        node.pos.y + current_chunk.max.x,
                                        node.pos.x + current_chunk.min.y,
                                        node.pos.z + current_chunk.min.z,
                                    ),
                                    Dir::SOUTH => UVec3::new(
                                        node.pos.x + current_chunk.min.x,
                                        node.pos.y + current_chunk.min.y,
                                        node.pos.z + current_chunk.min.z,
                                    ),
                                    Dir::WEST => UVec3::new(
                                        node.pos.y + current_chunk.min.x,
                                        node.pos.x + current_chunk.min.y,
                                        node.pos.z + current_chunk.min.z,
                                    ),
                                    // TODO: WE NEED TO FIX UP AND DOWN NODE STUFFS
                                    Dir::UP => UVec3::new(
                                        node.pos.x + current_chunk.min.x,
                                        node.pos.y + current_chunk.min.y,
                                        node.pos.z + current_chunk.max.z,
                                    ),
                                    Dir::DOWN => UVec3::new(
                                        node.pos.x + current_chunk.min.x,
                                        node.pos.y + current_chunk.min.y,
                                        node.pos.z + current_chunk.min.z,
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
                                        current_chunk.max.x,
                                        current_chunk.max.y,
                                        current_chunk.min.z,
                                    ),
                                    Dir::SOUTHEAST => UVec3::new(
                                        current_chunk.max.x,
                                        current_chunk.min.y,
                                        current_chunk.min.z,
                                    ),
                                    Dir::SOUTHWEST => UVec3::new(
                                        current_chunk.min.x,
                                        current_chunk.min.y,
                                        current_chunk.min.z,
                                    ),
                                    Dir::NORTHWEST => UVec3::new(
                                        current_chunk.min.x,
                                        current_chunk.max.y,
                                        current_chunk.min.z,
                                    ),
                                    Dir::NORTHEASTUP => UVec3::new(
                                        current_chunk.max.x,
                                        current_chunk.max.y,
                                        current_chunk.max.z,
                                    ),
                                    Dir::SOUTHEASTUP => UVec3::new(
                                        current_chunk.max.x,
                                        current_chunk.min.y,
                                        current_chunk.max.z,
                                    ),
                                    Dir::SOUTHWESTUP => UVec3::new(
                                        current_chunk.min.x,
                                        current_chunk.min.y,
                                        current_chunk.max.z,
                                    ),
                                    Dir::NORTHWESTUP => UVec3::new(
                                        current_chunk.min.x,
                                        current_chunk.max.y,
                                        current_chunk.max.z,
                                    ),
                                    Dir::NORTHEASTDOWN => UVec3::new(
                                        current_chunk.max.x,
                                        current_chunk.max.y,
                                        current_chunk.min.z,
                                    ),
                                    Dir::SOUTHEASTDOWN => UVec3::new(
                                        current_chunk.max.x,
                                        current_chunk.min.y,
                                        current_chunk.min.z,
                                    ),
                                    Dir::SOUTHWESTDOWN => UVec3::new(
                                        current_chunk.min.x,
                                        current_chunk.min.y,
                                        current_chunk.min.z,
                                    ),
                                    Dir::NORTHWESTDOWN => UVec3::new(
                                        current_chunk.min.x,
                                        current_chunk.max.y,
                                        current_chunk.min.z,
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

                    let nodes = self.graph.get_all_nodes_in_chunk(chunk.clone());

                    let mut connections = Vec::new();

                    for node in nodes.iter() {
                        // Collect other nodes positions into an array
                        let other_nodes = nodes
                            .iter()
                            .filter(|other_node| other_node.pos != node.pos)
                            .map(|other_node| other_node.pos)
                            .collect::<Vec<_>>();

                        // Adjust node.pos by the chunk position
                        let start_pos = node.pos - chunk.min;

                        let goals = other_nodes
                            .iter()
                            .map(|pos| *pos - chunk.min)
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
                            let goal = goal_pos + chunk.min;

                            // Readjust path to world space
                            let path_vec = path
                                .path()
                                .iter()
                                .map(|pos| *pos + chunk.min)
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

    fn connect_adjacent_chunk_nodes(&mut self) {
        let nodes = self.graph.get_nodes();

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

                if let Some(neighbor) = self.graph.get_node(UVec3::new(nx as u32, ny as u32, nz as u32)) {
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

    /// Returns a vector of `Node`s in the chunk for the given edge.
    /// 
    /// # Arguments
    /// 
    /// * `chunk` - The chunk to get the nodes from.
    /// 
    /// * `dir` - `Dir` direction of the edge.
    pub fn get_nodes_in_chunk_by_dir(&self, chunk: Chunk, dir: Dir) -> Vec<Node> {
        let mut nodes = Vec::new();

        let chunk_nodes = self.graph.get_all_nodes_in_chunk(chunk);

        for node in chunk_nodes {
            if node.dir == Some(dir) {
                nodes.push(node.clone());
            }
        }

        nodes
    }

    /// Returns the `Chunk` for the given position `UVec3` in the grid.
    pub fn get_chunk_for_position(&self, pos: UVec3) -> Option<&Chunk> {
        for chunk in self.chunks.iter() {
            if pos.x >= chunk.min.x
                && pos.x <= chunk.max.x
                && pos.y >= chunk.min.y
                && pos.y <= chunk.max.y
                && pos.z >= chunk.min.z
                && pos.z <= chunk.max.z
            {
                return Some(chunk);
            }
        }

        None
    }

    pub fn optimize_path_cardinal(&self, path: Path) -> Path {
        if path.is_empty() {
            return path;
        }

        let goal = *path.path.back().unwrap();

        let mut refined_path = Vec::new();
        let mut i = 0;

        refined_path.push(path.path[i]); // Always keep the first node

        while i < path.len() {
            // Check if we can go directly to the goal
            if line_of_sight_cardinal(&self.grid.view(), path.path[i], goal) {
                let segment = generate_path_segment_cardinal(path.path[i], goal);
                refined_path.extend(segment.into_iter().skip(1)); // Add intermediate points
                break; // We're done
            }

            let mut best_farthest = i + 1;
            let mut best_score = f32::NEG_INFINITY;

            for farthest in (i + 1)..path.len() {
                if line_of_sight_cardinal(&self.grid.view(), path.path[i], path.path[farthest]) {
                    let shortcut_length = farthest - i; // Reward longer shortcuts
                    let goal_proximity = self.neighborhood.heuristic(path.path[farthest], goal) as f32; // Reward shortcuts closer to the goal

                    // Base score: encourage longer shortcuts
                    let score = shortcut_length as f32 - goal_proximity * 4.0;
                    
                    // Pick the best shortcut that maximizes the score
                    if score > best_score {
                        best_farthest = farthest;
                        best_score = score;
                    }
                }
            }

            let segment = generate_path_segment_cardinal(path.path[i], path.path[best_farthest]);
            refined_path.extend(segment.into_iter().skip(1)); // Avoid duplicate start points
            i = best_farthest; // Move to the best shortcut

        }

        // Calculate the cost of the refined path
        let mut cost = 0;
        for pos in refined_path.iter() {
            cost += self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].cost;
        }
        let mut path = Path::new(refined_path.iter().cloned().collect(), cost);
        path.graph_path = path.path.clone();
        path
    }

    /// Optimize a path by using line of sight checks to skip waypoints.
    /// 
    /// This is used to optimize paths generated by the HPA* algorithms.
    /// `get_path` uses this internally so this is only needed if you want to
    /// optimize a path that was generated by a different method.
    pub fn optimize_path_ordinal(&self, path: Path) -> Path {
        if path.is_empty() {
            return path;
        }

        let goal = *path.path.back().unwrap();

        let mut refined_path = Vec::new();
        let mut i = 0;
        let mut prev_movement_type = None;

        refined_path.push(path.path[i]); // Always keep the first node

        while i < path.len() {    
            // Check if we can go directly to the goal
            if line_of_sight_ordinal(&self.grid.view(), path.path[i], goal) {
                let segment = generate_path_segment_ordinal(path.path[i], goal);
                refined_path.extend(segment.into_iter().skip(1)); // Add intermediate points
                break; // We're done
            }

            let mut best_farthest = i + 1;
            let mut best_score = f32::NEG_INFINITY;

            for farthest in (i + 1)..path.len() {
                if line_of_sight_ordinal(&self.grid.view(), path.path[i], path.path[farthest]) {
                    let movement_type = get_movement_type(path.path[i].as_vec3(), path.path[farthest].as_vec3());

                    let shortcut_length = farthest - i; // Reward longer shortcuts
                    let goal_proximity = self.neighborhood.heuristic(path.path[farthest], goal) as f32; // Reward shortcuts closer to the goal

                    // Base score: encourage longer shortcuts
                    let mut score = shortcut_length as f32 - goal_proximity * 4.0;

                    // Apply penalty if switching movement types
                    if let Some(prev) = prev_movement_type {
                        if prev != movement_type {
                            score -= 4.0; // Penalize changing movement types
                        }
                    }
                    
                    // Pick the best shortcut that maximizes the score
                    if score > best_score {
                        best_farthest = farthest;
                        best_score = score;
                    }
                }
            }

            let segment = generate_path_segment_ordinal(path.path[i], path.path[best_farthest]);
            refined_path.extend(segment.into_iter().skip(1)); // Avoid duplicate start points

            prev_movement_type = Some(get_movement_type(path.path[i].as_vec3(), path.path[best_farthest].as_vec3()));
    
            i = best_farthest; // Move to the best shortcut
        }

        // Calculate the cost of the refined path
        let mut cost = 0;

        for pos in refined_path.iter() {
            cost += self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]].cost;
        }

        let mut path = Path::new(refined_path.iter().cloned().collect(), cost);
        path.graph_path = path.path.clone();

        path   
    }

    /// Recursively reroutes a path by astar pathing to further chunks until a path can be found.
    /// 
    /// Useful if local collision avoidance is failing.
    /// 
    /// If you're using the plugin pathing systems, you shouldn't need to call this directly.
    /// 
    /// # Arguments
    pub fn reroute_path(&self, path: &Path, start: UVec3, goal: UVec3, blocking: &HashMap<UVec3, Entity>) -> Option<Path> {
        // When the starting chunks entrances are all blocked, this will try astar path to the NEXT chunk in the graph path
        // recursively until it can find a path out.
        // If it can't find a path out, it will return None.

        if path.graph_path.is_empty() {
            // Our only option here is to astar path to the goal
            return self.get_astar_path(start, goal, blocking, false);
        }

        let new_path = path.graph_path.iter().find_map(|pos| {
            let new_path = self.get_astar_path(start, *pos, blocking, false);
            if new_path.is_some() && new_path.as_ref().unwrap().len() > 0 {
                new_path
            } else {
                None
            }
        });

        // HPA the rest of the way to the goal using get_path from the last position in the new path to the goal
        if let Some(new_path) = new_path {
            let mut hpa_path = Vec::new();

            for pos in new_path.path() {
                hpa_path.push(*pos);
            }

            let last_pos = *new_path.path().last().unwrap();

            let hpa = self.get_path(last_pos, goal, blocking, false);

            if let Some(hpa) = hpa {
                for pos in hpa.path() {
                    hpa_path.push(*pos);
                }

                let mut path = Path::new(hpa_path, new_path.cost() + hpa.cost());
                path.graph_path = hpa.graph_path.clone();
                return Some(path);
            }
        }

        None
    }

    pub fn get_path(&self, start: UVec3, goal: UVec3, blocking: &HashMap<UVec3, Entity>, partial: bool) -> Option<Path> {
        if self.grid[[start.x as usize, start.y as usize, start.z as usize]].wall
            || self.grid[[goal.x as usize, goal.y as usize, goal.z as usize]].wall
        {
            return None;
        }

        let start_chunk = self.get_chunk_for_position(start)?;
        let goal_chunk = self.get_chunk_for_position(goal)?;

        if start_chunk == goal_chunk {
            let path = astar_grid(&self.neighborhood, &self.grid.view(), start, goal, 100, partial, blocking);
            
            if let Some(mut path) = path {
                // We should still refine this path since AStar wasn't requested
                path = if self.neighborhood.is_ordinal() {
                    self.optimize_path_ordinal(path)
                } else {
                    self.optimize_path_cardinal(path)
                };
                
                path.path.pop_front();
                return Some(path);
            } else {
                return None;
            }
        }

        // Get all nodes in the start chunk
        let start_nodes = self.graph.get_all_nodes_in_chunk(start_chunk.clone());

        // Build a new blocking map that's adjusted to the start chunk
        let start_blocking = blocking
            .iter()
            .map(|(pos, entity)| {
                let adjusted_pos = UVec3::new(
                    pos.x.saturating_sub(start_chunk.min.x),
                    pos.y.saturating_sub(start_chunk.min.y),
                    pos.z.saturating_sub(start_chunk.min.z),
                );
                (adjusted_pos, *entity)
            })
            .collect::<HashMap<_, _>>();

        // Get the djikstra paths to all starting nodes, ruling any that don't have paths
        let start_paths = dijkstra_grid(
            &self.neighborhood,
            &start_chunk.view(&self.grid),
            start - start_chunk.min,
            &start_nodes.iter().map(|node| node.pos - start_chunk.min).collect::<Vec<_>>(),
            false,
            100,
            &start_blocking,
        );

        // Rule out any start_nodes that don't have paths
        let start_nodes = start_nodes
            .iter()
            .filter(|node| start_paths.contains_key(&(node.pos - start_chunk.min)))
            .collect::<Vec<_>>();

        if start_nodes.is_empty() {
            return None;
        }


        // Sort start nodes by distance to the goal and the starting point
        let mut start_nodes = start_nodes
            .iter()
            .map(|node| {
                let distance_to_goal = (node.pos.x as i32 - goal.x as i32).abs()
                    + (node.pos.y as i32 - goal.y as i32).abs()
                    + (node.pos.z as i32 - goal.z as i32).abs();

                let distance_to_start = (node.pos.x as i32 - start.x as i32).abs()
                    + (node.pos.y as i32 - start.y as i32).abs()
                    + (node.pos.z as i32 - start.z as i32).abs();

                (node, distance_to_goal + distance_to_start)
                })
            .collect::<Vec<_>>();

        // Starting with the node shortest to the goal, find a path to the goal
        start_nodes.sort_by_key(|(_, distance)| *distance);

        // Get all nodes in the goal chunk
        let goal_nodes = self.graph.get_all_nodes_in_chunk(goal_chunk.clone());

        // Build a new blocking map that's adjusted for the goal chunk
        /*let goal_blocking = blocking.clone()
            .iter()
            .map(|(pos, entity)| {
                let adjusted_pos = UVec3::new(
                    pos.x.saturating_sub(goal_chunk.min.x),
                    pos.y.saturating_sub(goal_chunk.min.y),
                    pos.z.saturating_sub(goal_chunk.min.z),
                );
                (adjusted_pos, *entity)
            })
            .collect::<HashMap<_, _>>();*/

        let goal_paths = dijkstra_grid(
            &self.neighborhood,
            &goal_chunk.view(&self.grid),
            goal - goal_chunk.min,
            &goal_nodes.iter().map(|node| node.pos - goal_chunk.min).collect::<Vec<_>>(),
            false,
            100,
            &HashMap::new()//&goal_blocking,
        );

        let goal_nodes = goal_nodes
            .iter()
            .filter(|node| goal_paths.contains_key(&(node.pos - goal_chunk.min)))
            .collect::<Vec<_>>();

        let mut goal_nodes = goal_nodes
            .iter()
            .map(|node| {
                // Calculate the distance from the start to the node and from the node to the goal
                let distance_to_start = (node.pos.x as i32 - start.x as i32).abs()
                    + (node.pos.y as i32 - start.y as i32).abs()
                    + (node.pos.z as i32 - start.z as i32).abs();
                let distance_to_goal = (node.pos.x as i32 - goal.x as i32).abs()
                    + (node.pos.y as i32 - goal.y as i32).abs()
                    + (node.pos.z as i32 - goal.z as i32).abs();

                // Calculate the total distance as the sum of the distances to the start and goal
                let total_distance = distance_to_start + distance_to_goal;

                (node, total_distance)
            })
            .collect::<Vec<_>>();

        goal_nodes.sort_by_key(|(_, distance)| *distance);

        // Calculate the distance from the start to the goal
        let start_distance = (start.x as i32 - goal.x as i32).abs()
            + (start.y as i32 - goal.y as i32).abs()
            + (start.z as i32 - goal.z as i32).abs();

        // Move goal_nodes that are farther from the goal in the direction of the starting point to the end of the list
        goal_nodes.sort_by_key(|(_, distance)| {
            let distance = *distance as i32 - start_distance;
            if distance < 0 {
                distance.abs()
            } else {
                distance
            }
        });

        let mut path: Vec<UVec3> = Vec::new();
        let mut cost = 0;


        // Here is the problem. If the starting position can't get to the start node then this whole thing fails. Need to fix it.

        for (start_node, _) in start_nodes {
            for (goal_node, _) in goal_nodes.clone() {
                let node_path = astar_graph(
                    &self.neighborhood,
                    &self.graph,
                    start_node.pos,
                    goal_node.pos,
                    100,
                );

                if let Some(node_path) = node_path {
                    // Add start_path to the node_path
                    let start_path = start_paths.get(&(start_node.pos - start_chunk.min)).unwrap();
                    path.extend(start_path.path().iter().map(|pos| *pos + start_chunk.min));
                    cost += start_path.cost();


                    // Add node_path paths to path
                    for (node, next_node) in node_path.path().iter().zip(node_path.path().iter().skip(1)) {
                        // Get the cached edge path between node and next node
                        let cached_path = self.graph.get_node(*node).unwrap().edges[next_node].clone();
                        path.extend(cached_path.path().iter().skip(1));
                        cost += cached_path.cost();
                    }

                    // Add end path to path
                    let end_path = goal_paths.get(&(goal_node.pos - goal_chunk.min)).unwrap();
                    path.extend(end_path.path().iter().rev().map(|pos| *pos + goal_chunk.min));
                    cost += end_path.cost();

                    if path.len() == 0 {
                        return None;
                    }

                    let mut refined_path = if self.neighborhood.is_ordinal() {
                        self.optimize_path_ordinal(Path::new(path, cost))
                    } else {
                        self.optimize_path_cardinal(Path::new(path, cost))
                    };

                    // remove the start point from the refined path
                    refined_path.path.pop_front();

                    // add the graph path to the refined path
                    refined_path.graph_path = node_path.path;

                    return Some(refined_path);
                }
            }
        }

        None
    }

    pub fn get_astar_path(&self, start: UVec3, goal: UVec3, blocking: &HashMap<UVec3, Entity>, partial: bool) -> Option<Path> {
        if self.grid[[start.x as usize, start.y as usize, start.z as usize]].wall
            || self.grid[[goal.x as usize, goal.y as usize, goal.z as usize]].wall
        {
            return None;
        }

        // if goal is in the blocking map, return None
        if blocking.contains_key(&goal) && !partial {
            //log::error!("Goal is in the blocking map");
            return None;
        }

        let path = astar_grid(&self.neighborhood, &self.grid.view(), start, goal, 1024, partial, blocking);
        
        if let Some(mut path) = path {
            path.path.pop_front();
            return Some(path);
        } else {
            return None;
        }
    }
}

#[cfg(test)]
mod tests {
    use bevy::{math::UVec3, utils::hashbrown::HashMap};

    use crate::{
        dir::Dir,
        grid::{Grid, GridSettings},
        neighbor::OrdinalNeighborhood3d,
        Point,
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

        let nodes = grid.graph.get_nodes();

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
            .get_node(UVec3::new(2, 3, 0))
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
        let chunk = grid.get_chunk_for_position(UVec3::new(0, 0, 0)).unwrap();

        assert_eq!(chunk.min, UVec3::new(0, 0, 0));
        assert_eq!(
            chunk.max,
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

        let nodes = grid
            .graph
            .get_all_nodes_in_chunk(grid.chunks[[0, 0, 0]].clone());

        assert_eq!(nodes.len(), 2);
    }

    #[test]
    pub fn test_get_path() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = grid.get_path(UVec3::new(10, 10, 0), UVec3::new(4, 4, 0), &HashMap::new(), false);
        let raw_path = grid.get_astar_path(UVec3::new(10, 10, 0), UVec3::new(4, 4, 0), &HashMap::new(), false);

        assert!(path.is_some());
        // Ensure start point is the first point in the path
        assert_ne!(path.clone().unwrap().path()[0], UVec3::new(10, 10, 0));
        // Ensure end point is the last point in the path
        assert_eq!(path.clone().unwrap().path().last().unwrap(), &UVec3::new(4, 4, 0));

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

        let path = grid.get_path(UVec3::new(7, 7, 0), UVec3::new(121, 121, 0), &HashMap::new(), false);

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
        };

        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

        grid.build();
        let path = grid.get_path(UVec3::new(0, 0, 0), UVec3::new(31, 31, 3), &HashMap::new(), false);

        assert!(path.is_some());
    }

    #[test]
    pub fn test_get_astar_path() {
        let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = grid.get_astar_path(UVec3::new(0, 0, 0), UVec3::new(10, 10, 0), &HashMap::new(), false);

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 10);
    }
}
