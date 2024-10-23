use criterion::{criterion_group, criterion_main, Criterion};

use bevy_northstar::grid::{Grid, GridSettings};

mod profiler;


fn benchmarks(c: &mut Criterion) {
    let mut group = c.benchmark_group("pathfinding");

    let grid_settings = GridSettings {
        width: 64,
        height: 64,
        depth: 1,
        chunk_depth: 1,
        chunk_size: 32,
        default_cost: 1,
        default_wall: false,
        jump_height: 1,
    };

    let mut grid = Grid::new(&grid_settings);

    group.sample_size(10);
    group.bench_function("init_grid_64x64", |b| b.iter(|| 
        grid = Grid::new(&grid_settings)
    ));
    group.bench_function("build_grid_64x64", |b| b.iter(|| 
        grid.build()
    ));

    let grid_settings = GridSettings {
        width: 512,
        height: 512,
        depth: 1,
        chunk_depth: 1,
        chunk_size: 32,
        default_cost: 1,
        default_wall: false,
        jump_height: 1,
    };

    let mut grid = Grid::new(&grid_settings);

    group.bench_function("init_grid_512x512", |b| b.iter(|| 
        grid = Grid::new(&grid_settings)
    ));
    group.bench_function("build_grid_512x512", |b| b.iter(|| 
        grid.build()
    ));


    let grid_settings = GridSettings {
        width: 1024,
        height: 1024,
        depth: 1,
        chunk_depth: 1,
        chunk_size: 32,
        default_cost: 1,
        default_wall: false,
        jump_height: 1,
    };

    let mut grid = Grid::new(&grid_settings);

    group.bench_function("init_grid_1024x1024", |b| b.iter(|| 
        grid = Grid::new(&grid_settings)
    ));
    group.bench_function("build_grid_1024x1024", |b| b.iter(|| 
        grid.build()
    ));


    group.finish();
}

criterion_group!{
    name = benches;
    config = Criterion::default().with_profiler(profiler::FlamegraphProfiler::new(100)).sample_size(10);
    targets = benchmarks
}

criterion_main!(benches);

