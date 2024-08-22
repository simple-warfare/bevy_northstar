use criterion::{criterion_group, criterion_main, Criterion};
use bevy_northstar::{GridPosition, Pathfinding, PathfindingError};

fn benchmarks(c: &mut Criterion) {
    let mut group = c.benchmark_group("pathfinding");

    let mut pathfinding = Pathfinding::new(64, 64, 4, 1, true);

    group.sample_size(10);
    group.bench_function("init_grid", |b| b.iter(|| pathfinding.init_grid(1, true) ));
    group.bench_function("bench_pathfinding", |b| b.iter(||
        assert_ne!(pathfinding.get_path(&GridPosition::new(0, 0, 0), &GridPosition::new(63, 63, 3)), Err(PathfindingError::NoPathToGoal))
    ));
    group.finish();
}

criterion_group!(benches, benchmarks);
criterion_main!(benches);