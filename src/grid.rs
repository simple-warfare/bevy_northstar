use bevy::{math::UVec3, prelude::Resource};
use ndarray::{Array3, ArrayView2};

use crate::{
    astar::{astar_graph, astar_grid},
    chunk::Chunk,
    dijkstra::dijkstra_grid,
    dir::Dir,
    graph::Graph,
    neighbor::OrdinalNeighborhood3d,
    node::Node,
    path::Path,
    Point,
};

pub struct GridSettings {
    pub width: u32,
    pub height: u32,
    pub depth: u32,

    pub chunk_size: u32,
    pub chunk_depth: u32,

    pub default_cost: u32,
    pub default_wall: bool,

    pub jump_height: u32,
}

#[derive(Resource)]
pub struct Grid {
    width: u32,
    height: u32,
    depth: u32,

    chunk_size: u32,
    chunk_depth: u32,

    #[allow(dead_code)]
    default_cost: u32,
    #[allow(dead_code)]
    default_wall: bool,

    grid: Array3<Point>,
    pub chunks: Array3<Chunk>,

    pub graph: Graph,
}

impl Grid {
    pub fn new(settings: &GridSettings) -> Self {
        let x_chunks = settings.width as usize / settings.chunk_size as usize;
        let y_chunks = settings.height as usize / settings.chunk_size as usize;
        let z_chunks = settings.depth as usize / settings.chunk_depth as usize;

        Grid {
            width: settings.width,
            height: settings.height,
            depth: settings.depth,
            chunk_size: settings.chunk_size,
            chunk_depth: settings.chunk_depth,
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
                let max_x = min_x + settings.chunk_size;
                let min_y = y as u32 * settings.chunk_size;
                let max_y = min_y + settings.chunk_size;
                let min_z = z as u32 * settings.chunk_depth;
                let max_z = min_z + settings.chunk_depth;

                Chunk::new(
                    UVec3::new(min_x, min_y, min_z),
                    UVec3::new(max_x, max_y, max_z),
                )
            }),
            graph: Graph::new(),
        }
    }

    pub fn set_point(&mut self, pos: UVec3, point: Point) {
        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]] = point;
    }

    pub fn get_point(&self, pos: UVec3) -> Point {
        self.grid[[pos.x as usize, pos.y as usize, pos.z as usize]]
    }

    pub fn get_width(&self) -> u32 {
        self.width
    }

    pub fn get_height(&self) -> u32 {
        self.height
    }
    
    pub fn get_depth(&self) -> u32 {
        self.depth
    }

    pub fn get_chunk_size(&self) -> u32 {
        self.chunk_size
    }

    pub fn build(&mut self) {
        self.build_nodes();
        self.connect_internal_chunk_nodes();
        self.connect_adjacent_chunk_nodes();
    }

    pub fn build_nodes(&mut self) {
        let chunk_size = self.chunk_size as usize;
        let chunk_depth = self.chunk_depth as usize;

        let x_chunks = self.width as usize / chunk_size;
        let y_chunks = self.height as usize / chunk_size;
        let z_chunks = self.depth as usize / chunk_depth;

        for x in 0..x_chunks {
            for y in 0..y_chunks {
                for z in 0..z_chunks {
                    let current_chunk = &self.chunks[[x, y, z]];

                    let directions = Dir::all();

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

                            // Position the nodes in world space using the fixed axis to realign the nodes, they also need to be adjusted by the position of the chunk in the grid
                            for node in nodes.iter_mut() {
                                node.pos = match dir {
                                    Dir::NORTH => UVec3::new(
                                        node.pos.x + current_chunk.min.x,
                                        node.pos.y + current_chunk.max.y - 1,
                                        node.pos.z + current_chunk.min.z,
                                    ),
                                    Dir::EAST => UVec3::new(
                                        node.pos.y + current_chunk.max.x - 1,
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
                                        node.pos.z + current_chunk.max.z - 1,
                                    ),
                                    Dir::DOWN => UVec3::new(
                                        node.pos.x + current_chunk.min.x,
                                        node.pos.y + current_chunk.min.y,
                                        node.pos.z + current_chunk.min.z,
                                    ),
                                }
                            }

                            self.graph.add_nodes(&nodes);
                        }
                    }
                }
            }
        }

        // We shouldn't need this
        //self.graph.clear_duplicates();
    }

    pub fn calculate_edge_nodes(
        &self,
        start_edge: ArrayView2<Point>,
        end_edge: ArrayView2<Point>,
        chunk: Chunk,
        dir: Dir,
    ) -> Vec<Node> {
        let mut nodes = Vec::new();

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

            if !end_point.wall {
                let pos = UVec3::new(start_x as u32, start_y as u32, 0);

                let node = Node::new(pos, chunk.clone(), Some(dir));
                nodes.push(node);
            }
        }

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

        final_nodes
    }

    pub fn connect_internal_chunk_nodes(&mut self) {
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
                            OrdinalNeighborhood3d,
                            &chunk_grid,
                            start_pos,
                            &goals,
                            false,
                            100,
                        );

                        // Readjust position to world space and then connect the nodes
                        for (goal_pos, path) in paths {
                            let start = node.pos;
                            let goal = goal_pos + chunk.min;

                            // Readjust path to world space
                            let path_vec = path.path().iter().map(|pos| *pos + chunk.min).collect::<Vec<_>>();

                            connections.push((start, goal, Path::new(path_vec.clone(), path_vec.len() as u32)));
                        }
                    }

                    for (node_pos, other_node_pos, path) in connections {
                        self.graph.connect_node(node_pos, other_node_pos, path);
                    }
                }
            }
        }
    }

    pub fn connect_adjacent_chunk_nodes(&mut self) {
        let chunk_size = self.chunk_size as usize;
        let chunk_depth = self.chunk_depth as usize;

        let x_chunks = self.width as usize / chunk_size;
        let y_chunks = self.height as usize / chunk_size;
        let z_chunks = self.depth as usize / chunk_depth;

        for x in 0..x_chunks {
            for y in 0..y_chunks {
                for z in 0..z_chunks {
                    let current_chunk = &self.chunks[[x, y, z]];

                    let chunk_nodes = self.graph.get_all_nodes_in_chunk(current_chunk.clone());

                    let mut connections = Vec::new();

                    for node in chunk_nodes {
                        let direction = node.dir.unwrap();

                        let dir_vec = direction.vector();

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

                            let neighbor_nodes = self.get_nodes_in_chunk_by_dir(
                                neighbor_chunk.clone(),
                                direction.opposite(),
                            );

                            for neighbor_node in neighbor_nodes {
                                let path = astar_grid(
                                    OrdinalNeighborhood3d,
                                    &self.grid.view(),
                                    node.pos,
                                    neighbor_node.pos,
                                    8,
                                );

                                if let Some(path) = path {
                                    connections.push((node.pos, neighbor_node.pos, path));
                                }
                            }
                        }
                    }

                    for (node_pos, neighbor_node_pos, path) in connections {
                        self.graph.connect_node(node_pos, neighbor_node_pos, path);
                    }
                }
            }
        }
    }

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

    pub fn get_chunk_for_position(&self, pos: UVec3) -> Option<&Chunk> {
        for chunk in self.chunks.iter() {
            if pos.x >= chunk.min.x
                && pos.x < chunk.max.x
                && pos.y >= chunk.min.y
                && pos.y < chunk.max.y
                && pos.z >= chunk.min.z
                && pos.z < chunk.max.z
            {
                return Some(chunk);
            }
        }

        None
    }

    pub fn get_path(&self, start: UVec3, goal: UVec3) -> Option<Path> {
        if self.grid[[start.x as usize, start.y as usize, start.z as usize]].wall
            || self.grid[[goal.x as usize, goal.y as usize, goal.z as usize]].wall
        {
            return None;
        }

        let start_chunk = self.get_chunk_for_position(start)?;
        let goal_chunk = self.get_chunk_for_position(goal)?;

        if start_chunk == goal_chunk {
            return astar_grid(
                OrdinalNeighborhood3d,
                &self.grid.view(),
                start,
                goal,
                100,
            );
        }

        // Get all nodes in the start chunk
        let start_nodes = self.graph.get_all_nodes_in_chunk(start_chunk.clone());

        // Get all nodes in the goal chunk
        let goal_nodes = self.graph.get_all_nodes_in_chunk(goal_chunk.clone());

        // Sort start nodes by distance to the goal
        let mut start_nodes = start_nodes
            .iter()
            .map(|node| {
                let distance = (node.pos.x as i32 - goal.x as i32).abs()
                    + (node.pos.y as i32 - goal.y as i32).abs()
                    + (node.pos.z as i32 - goal.z as i32).abs();

                (node, distance)
            })
            .collect::<Vec<_>>();

        // Starting with the node shortest to the goal, find a path to the goal
        start_nodes.sort_by_key(|(_, distance)| *distance);

        // Sort goal nodes by distance to the goal
        let mut goal_nodes = goal_nodes
            .iter()
            .map(|node| {
                let distance = (node.pos.x as i32 - goal.x as i32).abs()
                    + (node.pos.y as i32 - goal.y as i32).abs()
                    + (node.pos.z as i32 - goal.z as i32).abs();

                (node, distance)
            })
            .collect::<Vec<_>>();

        goal_nodes.sort_by_key(|(_, distance)| *distance);

        let mut node_path: Option<Path> = None;

        for (start_node, _) in start_nodes {
            for (goal_node, _) in goal_nodes.clone() {
                node_path = astar_graph(
                    OrdinalNeighborhood3d,
                    &self.graph,
                    start_node.pos,
                    goal_node.pos,
                    100,
                );

                if node_path.is_some() {
                    break;
                }
            }
        }

        let mut path: Vec<UVec3> = Vec::new();
        let mut cost = 0;

        // If the node_path is found, find the path from the start to the first ppint in the node_path, then use the node cached paths to build the rest, finally find path from last node to goal
        if let Some(node_path) = node_path {
            let start_path = astar_grid(
                OrdinalNeighborhood3d,
                &self.grid.view(),
                start,
                node_path.path().first().unwrap().clone(),
                100,
            );

            if let Some(start_path) = start_path {
                path.extend(start_path.path());
                cost += start_path.cost();
            }

            // Find the path between all nodes in the node path
            for (node, next_node) in node_path.path().iter().zip(node_path.path().iter().skip(1)) {
                let current_path = astar_grid(
                    OrdinalNeighborhood3d,
                    &self.grid.view(),
                    node.clone(),
                    next_node.clone(),
                    100,
                );

                if let Some(current_path) = current_path {
                    path.extend(current_path.path().into_iter().skip(1));
                    cost += current_path.cost();
                }
            }

            let end_path = astar_grid(
                OrdinalNeighborhood3d,
                &self.grid.view(),
                node_path.path().last().unwrap().clone(),
                goal,
                100,
            );

            if let Some(end_path) = end_path {
                path.extend(end_path.path().into_iter().skip(1));
                cost += end_path.cost();
            }
        }

        Some(Path::new(path, cost))
    }
}

#[cfg(test)]
mod tests {
    use bevy::math::UVec3;

    use crate::{
        dir::Dir, grid::{Grid, GridSettings}, Point
    };

    const GRID_SETTINGS: GridSettings = GridSettings {
        width: 12,
        height: 12,
        depth: 1,
        chunk_size: 4,
        chunk_depth: 1,
        default_cost: 1,
        default_wall: false,
        jump_height: 1,
    };

    #[test]
    pub fn test_new() {
        let grid = Grid::new(&GRID_SETTINGS);
        assert_eq!(grid.grid.shape(), [12, 12, 1]);
    }

    #[test]
    pub fn test_calculate_edge_nodes_3d() {
        let grid = Grid::new(&GridSettings {
            width: 8,
            height: 8,
            depth: 8,
            chunk_size: 4,
            chunk_depth: 4,
            default_cost: 1,
            default_wall: false,
            jump_height: 1,
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
        let mut grid = Grid::new(&GRID_SETTINGS);

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

        let mut grid = Grid::new(&GRID_SETTINGS);

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
    }

    #[test]
    pub fn test_connect_internal_nodes() {
        let mut grid = Grid::new(&GRID_SETTINGS);

        grid.build_nodes();
        grid.connect_internal_chunk_nodes();

        assert_eq!(grid.graph.node_count(), 24);
        assert_eq!(grid.graph.edge_count(), 44);
    }

    #[test]
    pub fn test_connect_adjacent_nodes() {
        let mut grid = Grid::new(&GRID_SETTINGS);

        grid.build_nodes();
        grid.connect_adjacent_chunk_nodes();

        let edges = grid.graph.get_node(UVec3::new(2, 3, 0)).unwrap().edges.clone();

        assert_eq!(edges.contains_key(&UVec3::new(2, 4, 0)), true);

        assert_eq!(edges[&UVec3::new(2, 4, 0)].path().len(), 2);
        assert_eq!(edges[&UVec3::new(2, 4, 0)].cost(), 1);

        assert_eq!(grid.graph.edge_count(), 24);
    }

    #[test]
    pub fn test_get_chunk_for_position() {
        let grid = Grid::new(&GRID_SETTINGS);
        let chunk = grid.get_chunk_for_position(UVec3::new(0, 0, 0)).unwrap();

        assert_eq!(chunk.min, UVec3::new(0, 0, 0));
        assert_eq!(chunk.max, UVec3::new(GRID_SETTINGS.chunk_size, GRID_SETTINGS.chunk_size, GRID_SETTINGS.chunk_depth));
    }

    #[test]
    pub fn test_get_all_nodes_in_chunk() {
        let mut grid = Grid::new(&GRID_SETTINGS);
        grid.build_nodes();

        let nodes = grid
            .graph
            .get_all_nodes_in_chunk(grid.chunks[[0, 0, 0]].clone());

        assert_eq!(nodes.len(), 2);
    }

    #[test]
    pub fn test_get_path() {
        let mut grid = Grid::new(&GRID_SETTINGS);

        grid.build();

        let path = grid.get_path(UVec3::new(0, 0, 0), UVec3::new(10, 10, 0));

        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 16);
    }

    #[test]
    fn test_calculate_edge_nodes_returns_center() {
        let grid = Grid::new(&GridSettings {
            width: 64,
            height: 64,
            depth: 1,
            chunk_size: 32,
            chunk_depth: 1,
            default_cost: 0,
            default_wall: false,
            jump_height: 1,
        });

        let start_edge = grid.chunks[[0, 0, 0]].edge(&grid.grid, Dir::NORTH);
        let end_edge = grid.chunks[[0, 1, 0]].edge(&grid.grid, Dir::SOUTH);

        let nodes = grid.calculate_edge_nodes(start_edge, end_edge, grid.chunks[[0, 0, 0]].clone(), Dir::NORTH);

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

        let mut grid = Grid::new(&GridSettings {
            width,
            height,
            depth,
            chunk_size,
            chunk_depth,
            default_cost: 1,
            default_wall: false,
            jump_height: 1,
        });

        for x in 0..width {
            for y in 0..height {
                if (x % 4 == 0) && (y % chunk_size == 0 || y / chunk_size + chunk_size == chunk_size) {
                    grid.grid[[x as usize, y as usize, 0]] = Point::new(1, true);
                }

                if (y % 4 == 0) && (x % chunk_size == 0 || x & chunk_size + chunk_size == chunk_size) {
                    grid.grid[[x as usize, y as usize, 0]] = Point::new(1, true);
                }
            }
        }

        grid.build();

        let path = grid.get_path(UVec3::new(7, 7, 0), UVec3::new(121, 121, 0));

        assert!(path.is_some());
        assert!(path.unwrap().len() > 0);
    }
}
