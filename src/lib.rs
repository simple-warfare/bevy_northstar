use std::{collections::BinaryHeap, sync::{Arc, Mutex}};

use bevy::{prelude::*, utils::{hashbrown::HashMap, HashSet}};
use thiserror::Error;


#[derive(Default, Debug, Clone)]
struct Point {
    id: i64,
    pos: Vec3,

    weight: f32,
    enabled: bool,

    // Neighbors
    neighbors: HashSet<i64>,
    unlinked_neighbors: HashSet<i64>,

    prev_point: Option<i64>,

    //open_pass: u64,
    //closed_pass: u64,

    g_score: f32,
    f_score: f32,

    // Used for getting closest_point_of_last_pathing_call.
    abs_g_score: f32,
    abs_f_score: f32,
}

impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

impl Eq for Point {}

impl PartialOrd for Point {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        other.f_score.partial_cmp(&self.f_score)
    }
}

impl Ord for Point {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap().reverse()
    }
}

mod segment_direction {
    pub const NONE: u8 = 0;
    pub const FORWARD: u8 = 1;
    pub const BACKWARD: u8 = 2;
    pub const BIDIRECTIONAL: u8 = FORWARD | BACKWARD;
}

#[derive(Eq, Hash, PartialEq)]
struct Segment {
    key: (i64, i64),
    direction: u8,
}

#[derive(Default, Resource)]
pub struct Pathfinding {
    points: HashMap<i64, Arc<Mutex<Point>>>,
    segments: HashSet<Segment>,

    pass: u64,
}

// Errors
#[derive(Debug, Clone, Error)]
pub enum PathfindingError {
    #[error("Point does not exist")]
    PointDoesNotExist,
    #[error("IDs are equal")]
    IdsAreEqual,
    #[error("Bad index")]
    BadIndex,
    #[error("Path is unsolvable")]
    Unsolvable,
    #[error("Lib error")]
    LibError,
}

impl Pathfinding {
    pub fn add_vec(&mut self, pos: Vec3, weight: f32) {
        let index = Pathfinding::calculate_index(pos);
        self.add_point(index, pos, weight);
    }

    pub fn add_point(&mut self, id: i64, pos: Vec3, weight: f32) {
        if let Some(point_mutex) = self.points.get_mut(&id) {
            let mut point = point_mutex.lock().unwrap();
            point.pos = pos;
            point.weight = weight;
        } else {
            self.points.insert(id, Arc::new(Mutex::new(Point {
                id,
                pos,
                weight,
                enabled: true,
                ..Default::default()
            })));
        }
    }


    fn get_point(&self, id: &i64) -> Result<&Arc<Mutex<Point>>, PathfindingError> {
        let Some(point) = self.points.get(id) else {return Err(PathfindingError::PointDoesNotExist)};

        Ok(point)
    }

    pub fn connect_points(&mut self, from: Vec3, to: Vec3, bidirectional: bool) -> Result<(), PathfindingError> {
        let from_index = Pathfinding::calculate_index(from);
        let to_index = Pathfinding::calculate_index(to);

        self.connect_points_by_index(from_index, to_index, bidirectional)
    }

    pub fn connect_points_by_index(&mut self, from_id: i64, to_id: i64, bidirectional: bool) -> Result<(), PathfindingError> {
        if from_id == to_id {
            return Err(PathfindingError::IdsAreEqual);
        }

        let Some(from_mutex) = self.points.get(&from_id) else {return Err(PathfindingError::PointDoesNotExist)};
        let Some(to_mutex) = self.points.get(&to_id) else {return Err(PathfindingError::PointDoesNotExist)};

        let mut from = from_mutex.lock().unwrap();
        let mut to = to_mutex.lock().unwrap();

        from.neighbors.insert(to_id);

        if bidirectional {
            to.neighbors.insert(from_id);
        } else {
            to.unlinked_neighbors.insert(from_id);
        }

        let mut s = Segment { key: (from_id, to_id), direction: segment_direction::NONE };

        if bidirectional {
            s.direction = segment_direction::BIDIRECTIONAL;
        }

        if let Some(element) = self.segments.get(&s) {
            s.direction |= element.direction;

            if s.direction == segment_direction::BIDIRECTIONAL {
                to.unlinked_neighbors.remove(&from.id);
                from.unlinked_neighbors.remove(&to.id);
            }
        }

        self.segments.insert(s);

        Ok(())
    }

    pub fn calculate_index(pos: Vec3) -> i64 {
        let max_value: i64 = (1 << 20) - 1; // Maximum value that can be packed in 20 bits.
        let x = (pos.x as i64) & max_value;
        let y = (pos.y as i64) & max_value;
        let z = (pos.z as i64) & max_value;
    
        (x << 40) | (y << 20) | z
    }

    pub fn get_point_path_by_index(&mut self, start_index: i64, end_index: i64) -> Result<Vec<Vec3>, PathfindingError> {
        let mut path: Vec<Vec3> = Vec::new();

        if !self.points.contains_key(&start_index) || !self.points.contains_key(&end_index) {
            return Err(PathfindingError::BadIndex)
        }

        if start_index == end_index {
            return Err(PathfindingError::IdsAreEqual)
        }

        let maybe_last_point = self.solve(start_index, end_index)?;

        if let Some(last_point) = maybe_last_point {
            let mut point_arc = self.get_point(&last_point.id)?;
            let mut point = point_arc.lock().unwrap();

            while point.prev_point.is_some() {
                path.push(point.pos);

                let prev_point_id = point.prev_point.unwrap();
                point_arc = self.get_point(&prev_point_id)?;
                std::mem::drop(point);
                point = point_arc.lock().unwrap();
            }
        } else {
            return Err(PathfindingError::Unsolvable)
        }

        path.reverse();

        Ok(path)
    }

    pub fn get_point_path(&mut self, start: Vec3, end: Vec3) -> Result<Vec<Vec3>, PathfindingError> {
        let start_index = Pathfinding::calculate_index(start);
        let end_index = Pathfinding::calculate_index(end);

        self.get_point_path_by_index(start_index, end_index)
    }

    fn solve(&mut self, begin_index: i64, end_index: i64) -> Result<Option<Point>, PathfindingError> {
        if begin_index == end_index {
            return Ok(None);
        }

        self.pass += 1;

        let begin_arc = self.get_point(&begin_index)?;
        let end_arc = self.get_point(&end_index)?;

        let mut begin = begin_arc.lock().unwrap();
        let end_pos = {
            let end = end_arc.lock().unwrap();
            end.pos
        };

        let mut open_list: BinaryHeap<Point> = BinaryHeap::new();
        let mut open_list_ids: Vec<i64> = Vec::new();
        let mut closed_list: Vec<i64> = Vec::new();

        begin.f_score = begin.pos.distance(end_pos);
        begin.abs_f_score = begin.f_score;

        open_list.push(begin.clone());

        std::mem::drop(begin);

        let mut last_closest_point: Option<Point> = None;

        while !open_list.is_empty() {
            let point = open_list.pop().unwrap();

            if let Some(lcp) = last_closest_point.clone() {
                if lcp.abs_f_score > point.abs_f_score
                    || (lcp.abs_f_score >= point.abs_f_score && lcp.abs_g_score > point.abs_g_score) 
                {
                    last_closest_point = Some(point.clone());
                }
            } else {
                last_closest_point = Some(point.clone());
            }

            if point.id == end_index {
                last_closest_point = Some(point.clone());
                break;
            }

            closed_list.push(point.id);

            for neighbor_id in point.neighbors.clone().into_iter() {
                let Some(neighbor_arc) = self.points.get(&neighbor_id) else {return Err(PathfindingError::PointDoesNotExist)};
                let mut neighbor = neighbor_arc.lock().unwrap();

                if !neighbor.enabled || closed_list.contains(&neighbor.id) {
                    continue;
                }

                let distance = point.pos.distance(neighbor.pos);

                let tentative_g_score = point.g_score + distance * neighbor.weight;

                let mut new_point = false;

                if !open_list_ids.contains(&neighbor.id) {
                    new_point = true;
                    open_list_ids.push(neighbor_id);
                }
                else if tentative_g_score >= neighbor.g_score {
                    continue;
                }

                neighbor.prev_point = Some(point.id); 
                neighbor.g_score = tentative_g_score;
                neighbor.f_score = neighbor.g_score + distance;
                neighbor.abs_g_score = tentative_g_score;
                neighbor.abs_f_score = neighbor.f_score - neighbor.g_score;

                if new_point {
                    open_list.push(neighbor.clone());
                }

                std::mem::drop(neighbor);                               
            }

            std::mem::drop(point);
        }

        Ok(last_closest_point)
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BinaryHeap;

    use crate::Pathfinding;
    use bevy::math::Vec3;
    use crate::Point;

    #[test]
    fn test_point_ordering() {
        let mut a = Point::default();
        a.f_score = 1.0;
        let mut b = Point::default();
        b.f_score = 1.2;
        let mut c = Point::default();
        c.f_score = 1.4;

        let mut heap: BinaryHeap<&Point> = BinaryHeap::new();
        heap.push(&a);
        heap.push(&c);
        heap.push(&b);

        assert_eq!(a == b, false);
        assert_eq!(a < b, false);
        assert_eq!(heap.pop(), Some(&a));
    }

    #[test]
    fn test_calculate_index() {
        let idx1 = Pathfinding::calculate_index(Vec3::new(1.0, 0.0, 0.0));
        let idx2 = Pathfinding::calculate_index(Vec3::new(0.0, 1.0, 0.0));
        let idx3 = Pathfinding::calculate_index(Vec3::new(0.0, 0.0, 1.0));

        assert!(idx1 != idx2 && idx1 != idx3 && idx2 != idx3);
    }

    #[test]
    fn test_same_id() {
        let mut pathfinding = Pathfinding::default();

        pathfinding.add_point(0, Vec3::new(0.0, 0.0, 0.0), 1.0);

        assert_eq!(pathfinding.solve(0, 0).unwrap(), None);
    }

    #[test]
    fn test_solveable() {
        let mut pathfinding = Pathfinding::default();

        let v1 = Vec3::new(0.0, 0.0, 0.0);
        let i1 = Pathfinding::calculate_index(v1);
        let v2 = Vec3::new(1.0, 0.0, 0.0);
        let i2 = Pathfinding::calculate_index(v2);
        let v3 = Vec3::new(2.0, 0.0, 0.0);
        let i3 = Pathfinding::calculate_index(v3);


        pathfinding.add_point(i1, v1, 1.0);
        pathfinding.add_point(i2, v2, 1.0);
        pathfinding.add_point(i3, v3, 1.0);

        let _ = pathfinding.connect_points_by_index(i1, i2, true);
        let _ = pathfinding.connect_points_by_index(i2, i3, true);

        assert_ne!(pathfinding.solve(i1, i3).unwrap(), None);
    }

    #[test]
    fn test_unsolveable() {
        let mut pathfinding = Pathfinding::default();

        let v1 = Vec3::new(0.0, 0.0, 0.0);
        let i1 = Pathfinding::calculate_index(v1);
        let v2 = Vec3::new(1.0, 0.0, 0.0);
        let i2 = Pathfinding::calculate_index(v2);
        let v3 = Vec3::new(2.0, 0.0, 1.0);
        let i3 = Pathfinding::calculate_index(v3);
        let v4 = Vec3::new(3.0, 0.0, 0.0);
        let i4 = Pathfinding::calculate_index(v4);

        pathfinding.add_point(i1, v1, 1.0);
        pathfinding.add_point(i2, v2, 1.0);
        pathfinding.add_point(i3, v3, 1.0);
        pathfinding.add_point(i4, v4, 0.0);

        let _ = pathfinding.connect_points_by_index(i1, i2, false);
        let _ = pathfinding.connect_points_by_index(i2, i3, false);

        assert_ne!(pathfinding.solve(i1, i4).unwrap(), None);        
    }

    #[test]
    fn test_get_path() {
        let mut pathfinding = Pathfinding::default();

        let v1 = Vec3::new(0.0, 0.0, 0.0);
        let i1 = Pathfinding::calculate_index(v1);
        let v2 = Vec3::new(1.0, 0.0, 0.0);
        let i2 = Pathfinding::calculate_index(v2);
        let v3 = Vec3::new(2.0, 0.0, 0.0);
        let i3 = Pathfinding::calculate_index(v3);

        pathfinding.add_point(i1, v1, 1.0);
        pathfinding.add_point(i2, v2, 1.0);
        pathfinding.add_point(i3, v3, 1.0);

        let _ = pathfinding.connect_points_by_index(i1, i2, true);
        let _ = pathfinding.connect_points_by_index(i2, i3, true);

        let path = pathfinding.get_point_path(v1, v3).unwrap();

        let test = vec![v2, v3];

        assert_eq!(path, test);
    }

    #[test]
    fn test_weighting() {
        let path_one: Vec<Vec3>;
        let path_two: Vec<Vec3>;

        let mut pathfinding = Pathfinding::default();
        let mut grid: Vec<[(f32, i64); 4]> = vec![
            [(0.0, 0), (0.0, 1), (0.0, 2), (0.0, 3)],
            [(255.0, 4), (0.0, 5), (0.0, 6), (0.0, 7)],
            [(0.0, 8), (0.0, 9), (0.0, 10), (0.0, 11)],
            [(0.0, 12), (0.0, 13), (0.0, 14), (0.0, 15)],
        ];

        for x in 0..4 {
            for y in 0..4 {
                let vec = Vec3::new(x as f32, y as f32, 0.0);
                pathfinding.add_point(grid[x][y].1, vec, grid[x][y].0);
            }
        }

        for x in 0..4 {
            for y in 0..4 {
                if x > 0 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x-1][y].1, true).unwrap();
                }
                if x < 3 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x+1][y].1, true).unwrap();
                }
                if y > 0 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x][y-1].1, true).unwrap();
                }
                if y < 3 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x][y+1].1, true).unwrap();
                }
            }
        }

        path_one = pathfinding.get_point_path_by_index(0, 15).unwrap();

        grid[1][0].0 = 0.0;

        let mut pathfinding = Pathfinding::default();

        for x in 0..4 {
            for y in 0..4 {
                let vec = Vec3::new(x as f32, y as f32, 0.0);
                pathfinding.add_point(grid[x][y].1, vec, grid[x][y].0);
            }
        }

        for x in 0..4 {
            for y in 0..4 {
                if x > 0 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x-1][y].1, true).unwrap();
                }
                if x < 3 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x+1][y].1, true).unwrap();
                }
                if y > 0 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x][y-1].1, true).unwrap();
                }
                if y < 3 {
                    pathfinding.connect_points_by_index(grid[x][y].1, grid[x][y+1].1, true).unwrap();
                }
            }
        }        

        path_two = pathfinding.get_point_path_by_index(0, 15).unwrap();

        assert_ne!(path_one, path_two);
    }
}