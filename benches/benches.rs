use criterion::{criterion_group, criterion_main, Criterion};
use bevy_northstar::Pathfinding;
use bevy::math::Vec3;

fn corner_to_corner_64() {
    let mut pathfinding = Pathfinding::default();

    for x in 0..64 {
        for y in 0..64 {
            pathfinding.add_vec(Vec3::new(x as f32, y as f32, 0.0), 0.0);
        }
    }

    for x in 0..64 {
        for y in 0..64 {
            if x > 0 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32 - 1.0, y as f32, 0.), 
                    true
                ).unwrap();
            }
            if x < 63 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32 + 1.0, y as f32, 0.), 
                    true
                ).unwrap();
            }
            if y > 0 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32, y as f32 - 1.0, 0.), 
                    true
                ).unwrap();
            }
            if y < 63 {
                pathfinding.connect_points(
                    Vec3::new(x as f32, y as f32, 0.), 
                    Vec3::new(x as f32, y as f32 + 1.0, 0.), 
                    true
                ).unwrap();
            }
        }
    }

    let _ = pathfinding.get_point_path(
        Vec3::new(0., 0., 0.),
        Vec3::new(63., 63., 0.)
    );
}


fn benchmarks(c: &mut Criterion) {
    let mut group = c.benchmark_group("pathfinding");

    group.sample_size(10);
    group.bench_function("corner_to_corner_64", |b| b.iter(|| corner_to_corner_64() ));
    group.finish();
}

criterion_group!(benches, benchmarks);
criterion_main!(benches);