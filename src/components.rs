
use bevy::{math::UVec3, prelude::Component, prelude::Color};

#[derive(Component, Default, Debug, Clone, Eq, PartialEq, Hash)]
pub struct Position(pub UVec3);

#[derive(Component, Default)]
pub struct Blocking;

#[derive(Component, Default, Debug)]
pub struct Pathfind {
    pub goal: UVec3,
    pub use_astar: bool,
}

#[derive(Component, Default, Debug)]
pub struct Next(pub UVec3);

#[derive(Component, Default, Debug)]
pub struct AvoidanceFailed;

#[derive(Component, Default, Debug)]
pub struct RerouteFailed;

#[derive(Component, Default)]
pub struct DebugColor(pub Color);