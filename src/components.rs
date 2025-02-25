
use bevy::{math::UVec3, prelude::Component, prelude::Color};

#[derive(Component, Default, Debug, Clone, Eq, PartialEq, Hash)]
pub struct Position(pub UVec3);

#[derive(Component, Default)]
pub struct Blocking;

#[derive(Component)]
pub struct Goal(pub UVec3);

#[derive(Component, Default)]
pub struct Next(pub UVec3);

#[derive(Component, Default)]
pub struct DebugColor(pub Color);