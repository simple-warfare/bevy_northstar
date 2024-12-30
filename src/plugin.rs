use bevy::prelude::*;
use crate::prelude::*;

#[derive(Default)]
pub struct NorthstarPlugin<N: Neighborhood> {
    _neighborhood: std::marker::PhantomData<N>,
}



impl<N: 'static + Neighborhood> Plugin for NorthstarPlugin<N> {
    fn build(&self, app: &mut App) {
        app
            .add_systems(Update, update_pathfinders::<N>);
    }
}

fn update_pathfinders<N: Neighborhood>(
    grid: Res<Grid<N>>,
    mut query: Query<&mut PathFind, Changed<PathFind>>,
) where N: 'static + Neighborhood {
    query.par_iter_mut().for_each(|mut pathfind| {
        if pathfind.start == pathfind.end {
            return;
        }

        let path = grid.get_path(pathfind.start.to_uvec3(), pathfind.end.to_uvec3());
        pathfind.path = path;
    });
}