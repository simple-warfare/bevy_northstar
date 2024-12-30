use std::collections::VecDeque;

use bevy::{prelude::*, utils::hashbrown::HashSet};

use crate::{path::Path, GridPosition};

#[derive(Component, Default)]
pub struct PathFind {
    pub start: GridPosition,
    pub end: GridPosition,
    pub path: Option<Path>,
}

#[derive(Component, Default)]
pub struct PathBlocking;
