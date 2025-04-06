use bevy::{math::UVec3, utils::hashbrown::HashMap};
use criterion::{criterion_group, criterion_main, Criterion};

use bevy_northstar::{grid::{Grid, GridSettings}, prelude::{OrdinalNeighborhood, OrdinalNeighborhood3d}};

mod profiler;


fn benchmarks(c: &mut Criterion) {
    let mut group = c.benchmark_group("pathfinding");

    let grid_settings = GridSettings {
        width: 64,
        height: 64,
        depth: 1,
        chunk_size: 32,
        chunk_depth: 1,
        chunk_ordinal: false,
        default_cost: 1,
        default_wall: false,
    };

    let mut grid: Grid<OrdinalNeighborhood> = Grid::new(&grid_settings);

    group.sample_size(10);
    
    group.bench_function("build_grid_64x64", |b| b.iter(|| 
        grid.build()
    ));

    group.bench_function("pathfind_64x64", |b| b.iter(|| 
        grid.pathfind(UVec3::new(0, 0, 0), UVec3::new(63, 63, 0), &HashMap::new(), false)
    ));

    group.bench_function("raw_pathfind_64x64", |b| b.iter(|| 
        grid.pathfind_astar(UVec3::new(0, 0, 0), UVec3::new(63, 63, 0), &HashMap::new(), false)
    ));

    let grid_settings = GridSettings {
        width: 512,
        height: 512,
        depth: 1,
        chunk_depth: 1,
        chunk_size: 32,
        chunk_ordinal: false,
        default_cost: 1,
        default_wall: false,
    };

    let mut grid: Grid<OrdinalNeighborhood> = Grid::new(&grid_settings);

    group.bench_function("build_grid_512x512", |b| b.iter(|| 
        grid.build()
    ));

    group.bench_function("pathfind_512x512", |b| b.iter(|| 
        grid.pathfind(UVec3::new(0, 0, 0), UVec3::new(511, 511, 0), &HashMap::new(), false)
    ));

    group.bench_function("raw_pathfind_512x512", |b| b.iter(|| 
        grid.pathfind_astar(UVec3::new(0, 0, 0), UVec3::new(511, 511, 0), &HashMap::new(), false)
    ));


    let grid_settings = GridSettings {
        width: 128,
        height: 128,
        depth: 4,
        chunk_depth: 1,
        chunk_size: 16,
        chunk_ordinal: false,
        default_cost: 1,
        default_wall: false,
    };

    let mut grid: Grid<OrdinalNeighborhood3d> = Grid::new(&grid_settings);

    group.bench_function("build_grid_128x128x4", |b| b.iter(|| 
        grid.build()
    ));

    group.bench_function("pathfind_128x128x4", |b| b.iter(|| 
        grid.pathfind(UVec3::new(0, 0, 0), UVec3::new(127, 127, 3), &HashMap::new(), false)
    ));

    group.bench_function("raw_pathfind_128x128x4", |b| b.iter(|| 
        grid.pathfind_astar(UVec3::new(0, 0, 0), UVec3::new(127, 127, 3), &HashMap::new(), false)
    ));
    
    group.finish();
}

criterion_group!{
    name = benches;
    config = Criterion::default().with_profiler(profiler::FlamegraphProfiler::new(100)).sample_size(10);
    targets = benchmarks
}

criterion_main!(benches);

