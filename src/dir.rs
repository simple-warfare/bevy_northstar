#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dir {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
    UP = 4,
    DOWN = 5,
    NORTHEAST = 6,
    SOUTHEAST = 7,
    SOUTHWEST = 8,
    NORTHWEST = 9,
    NORTHEASTUP = 10,
    SOUTHEASTUP = 11,
    SOUTHWESTUP = 12,
    NORTHWESTUP = 13,
    NORTHEASTDOWN = 14,
    SOUTHEASTDOWN = 15,
    SOUTHWESTDOWN = 16,
    NORTHWESTDOWN = 17,
}

use bevy::math::{IVec3, Vec3};

pub use self::Dir::*;

impl Dir {
    pub fn cardinal() -> std::iter::Copied<std::slice::Iter<'static, Dir>> {
        [NORTH, EAST, SOUTH, WEST, UP, DOWN].iter().copied()
    }

    pub fn ordinal() -> std::iter::Copied<std::slice::Iter<'static, Dir>> {
        [
            NORTHEAST,
            SOUTHEAST,
            SOUTHWEST,
            NORTHWEST,
            NORTHEASTUP,
            SOUTHEASTUP,
            SOUTHWESTUP,
            NORTHWESTUP,
            NORTHEASTDOWN,
            SOUTHEASTDOWN,
            SOUTHWESTDOWN,
            NORTHWESTDOWN,
        ]
        .iter()
        .copied()
    }

    pub fn all() -> std::iter::Copied<std::slice::Iter<'static, Dir>> {
        [
            NORTH,
            EAST,
            SOUTH,
            WEST,
            UP,
            DOWN,
            NORTHEAST,
            SOUTHEAST,
            SOUTHWEST,
            NORTHWEST,
            NORTHEASTUP,
            SOUTHEASTUP,
            SOUTHWESTUP,
            NORTHWESTUP,
            NORTHEASTDOWN,
            SOUTHEASTDOWN,
            SOUTHWESTDOWN,
            NORTHWESTDOWN,
        ]
        .iter()
        .copied()
    }

    pub fn vector(self) -> (i32, i32, i32) {
        match self {
            NORTH => (0, 1, 0),
            EAST => (1, 0, 0),
            SOUTH => (0, -1, 0),
            WEST => (-1, 0, 0),
            UP => (0, 0, 1),
            DOWN => (0, 0, -1),
            NORTHEAST => (1, 1, 0),
            SOUTHEAST => (1, -1, 0),
            SOUTHWEST => (-1, -1, 0),
            NORTHWEST => (-1, 1, 0),
            NORTHEASTUP => (1, 1, 1),
            SOUTHEASTUP => (1, -1, 1),
            SOUTHWESTUP => (-1, -1, 1),
            NORTHWESTUP => (-1, 1, 1),
            NORTHEASTDOWN => (1, 1, -1),
            SOUTHEASTDOWN => (1, -1, -1),
            SOUTHWESTDOWN => (-1, -1, -1),
            NORTHWESTDOWN => (-1, 1, -1),
        }
    }

    pub fn get_direction(start: &Vec3, end: &Vec3) -> Self {
        let delta = (end - start).normalize_or_zero();

        let vec = IVec3::new(delta.x.round() as i32, delta.y.round() as i32, delta.z.round() as i32);

        match vec {
            IVec3 { x: 0, y: 1, z: 0 } => NORTH,
            IVec3 { x: 1, y: 0, z: 0 } => EAST,
            IVec3 { x: 0, y: -1, z: 0 } => SOUTH,
            IVec3 { x: -1, y: 0, z: 0 } => WEST,
            IVec3 { x: 0, y: 0, z: 1 } => UP,
            IVec3 { x: 0, y: 0, z: -1 } => DOWN,
            IVec3 { x: 1, y: 1, z: 0 } => NORTHEAST,
            IVec3 { x: 1, y: -1, z: 0 } => SOUTHEAST,
            IVec3 { x: -1, y: -1, z: 0 } => SOUTHWEST,
            IVec3 { x: -1, y: 1, z: 0 } => NORTHWEST,
            IVec3 { x: 1, y: 1, z: 1 } => NORTHEASTUP,
            IVec3 { x: 1, y: -1, z: 1 } => SOUTHEASTUP,
            IVec3 { x: -1, y: -1, z: 1 } => SOUTHWESTUP,
            IVec3 { x: -1, y: 1, z: 1 } => NORTHWESTUP,
            IVec3 { x: 1, y: 1, z: -1 } => NORTHEASTDOWN,
            IVec3 { x: 1, y: -1, z: -1 } => SOUTHEASTDOWN,
            IVec3 { x: -1, y: -1, z: -1 } => SOUTHWESTDOWN,
            IVec3 { x: -1, y: 1, z: -1 } => NORTHWESTDOWN,
            _ => panic!("Not a valid direction"),
        }
    }

    pub fn opposite(self) -> Dir {
        match self {
            NORTH => SOUTH,
            EAST => WEST,
            SOUTH => NORTH,
            WEST => EAST,
            UP => DOWN,
            DOWN => UP,
            NORTHEAST => SOUTHWEST,
            SOUTHEAST => NORTHWEST,
            SOUTHWEST => NORTHEAST,
            NORTHWEST => SOUTHEAST,
            NORTHEASTUP => SOUTHWESTDOWN,
            SOUTHEASTUP => NORTHWESTDOWN,
            SOUTHWESTUP => NORTHEASTDOWN,
            NORTHWESTUP => SOUTHEASTDOWN,
            NORTHEASTDOWN => SOUTHWESTUP,
            SOUTHEASTDOWN => NORTHWESTUP,
            SOUTHWESTDOWN => NORTHEASTUP,
            NORTHWESTDOWN => SOUTHEASTUP,
        }
    }

    pub fn fixed_axis(self) -> usize {
        match self {
            NORTH | SOUTH => 1,
            EAST | WEST => 0,
            UP | DOWN => 2,
            _ => panic!("Not a fixed axis direction"),
        }
    }

    pub fn is_verticle(self) -> bool {
        match self {
            UP | DOWN | NORTHEASTUP | SOUTHEASTUP | SOUTHWESTUP | NORTHWESTUP | NORTHEASTDOWN
            | SOUTHEASTDOWN | SOUTHWESTDOWN | NORTHWESTDOWN => true,
            _ => false,
        }
    }

    pub fn in_general_direction(self, other: &Dir) -> bool {
        // Convert self to a Vec3
        let self_vec = self.vector();
        let self_vec = Vec3::new(self_vec.0 as f32, self_vec.1 as f32, self_vec.2 as f32);

        // Convert other to a Vec3
        let other = other.vector();
        let other = Vec3::new(other.0 as f32, other.1 as f32, other.2 as f32);

        // Normalize the vectors
        let self_vec = self_vec.normalize();
        let other = other.normalize();

        // Get the dot product
        let dot = self_vec.dot(other);

        // If the dot product is greater than 0.5, then the vectors are in the same general direction
        dot > 0.5
    }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum MovementType {
    Cardinal,   // (x+1, y, z) or (x, y+1, z) etc.
    Diagonal2D, // (x+1, y+1, z) or (x+1, y, z+1) etc.
    Diagonal3D, // (x+1, y+1, z+1)
    None,       // Should never happen in normal cases
}

pub fn get_movement_type(a: Vec3, b: Vec3) -> MovementType {
    let dx = (b.x as isize - a.x as isize).abs();
    let dy = (b.y as isize - a.y as isize).abs();
    let dz = (b.z as isize - a.z as isize).abs();

    let num_changes = (dx > 0) as u8 + (dy > 0) as u8 + (dz > 0) as u8;

    match num_changes {
        1 => MovementType::Cardinal,   // Moving along one axis (x, y, or z)
        2 => MovementType::Diagonal2D, // Moving along two axes (XY, XZ, YZ)
        3 => MovementType::Diagonal3D, // Moving along all three axes (XYZ)
        _ => MovementType::None,       // No movement (shouldn't happen in valid paths)
    }
}
