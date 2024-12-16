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
}
