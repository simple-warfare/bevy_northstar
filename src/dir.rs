#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dir {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
    UP = 4,
    DOWN = 5,
}

pub use self::Dir::*;

impl Dir {
    pub fn all() -> std::iter::Copied<std::slice::Iter<'static, Dir>> {
        [NORTH, EAST, SOUTH, WEST, UP, DOWN].iter().copied()
    }

    pub fn vector(self) -> (i32, i32, i32) {
        match self {
            NORTH => (0, 1, 0),
            EAST => (1, 0, 0),
            SOUTH => (0, -1, 0),
            WEST => (-1, 0, 0),
            UP => (0, 0, 1),
            DOWN => (0, 0, -1),
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
        }
    }

    pub fn fixed_axis(self) -> usize {
        match self {
            NORTH | SOUTH => 1,
            EAST | WEST => 0,
            UP | DOWN => 2,
        }
    }

    pub fn is_verticle(self) -> bool {
        self == UP || self == DOWN
    }
}
