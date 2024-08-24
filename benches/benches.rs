use criterion::{criterion_group, criterion_main, Criterion};
use bevy_northstar::{GridPosition, Pathfinding, PathfindingError};
use rand::Rng;

fn benchmarks(c: &mut Criterion) {
    let mut group = c.benchmark_group("pathfinding");

    let mut pathfinding = Pathfinding::new(64, 64, 4, 1, true);

    group.sample_size(10);
    group.bench_function("init_grid", |b| b.iter(|| pathfinding.init_grid(1, true) ));
    group.bench_function("small_grid", |b| b.iter(||
        assert_ne!(pathfinding.get_path(&GridPosition::new(0, 0, 0), &GridPosition::new(63, 63, 3)), Err(PathfindingError::NoPathToGoal))
    ));

    let pathfinding = Pathfinding::new(4092, 4092, 16, 1, true);

    group.bench_function("huge_grid", |b| b.iter(|| 
        assert_ne!(pathfinding.get_path(&GridPosition::new(0,0,0), &GridPosition::new(4091, 4091, 15)), Err(PathfindingError::NoPathToGoal))
    ));

    let mut pathfinding = Pathfinding::new(64, 64, 1, 1, true);

    for y in 1..64 {
        pathfinding.disable_position(32, y, 0);
    }

    group.bench_function("mid_grid_with_wall", |b| b.iter(|| 
        assert_ne!(pathfinding.get_path(&GridPosition::new(0,0,0), &GridPosition::new(31, 31, 0)), Err(PathfindingError::NoPathToGoal))
    ));

    group.finish();
}

criterion_group!(benches, benchmarks);
criterion_main!(benches);