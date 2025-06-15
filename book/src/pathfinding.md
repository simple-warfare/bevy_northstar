# Pathfinding With Built-In Systems

## Pathfind and GridPos Components

To use the `NorthstarPlugin` pathfinding systems, insert a `Pathfind` and `GridPos` component.

The `GridPos` you will need to maintain as the current GridPos of the entity.

```rust,no_run
commands
    .spawn((
        Name::new("Player"),
        Pathfind {
            goal: UVec3::new(12, 12, 0),
            // you can set use_astar to true to bypass HPA* and use the traditional A* pathfinding algorithm if desired.
            use_astar: false,
        },
        GridPos(UVec3::new(1, 1, 0)),
        Blocking, // Insert the Blocking component if using collision and this entity should block.
    ));
```

There are also shortened functions for creating a new component using HPA* or A*

```rust,no_run
commands
    .spawn((
        Name::new("Player"),
        // new will default to HPA*.
        Pathfind::new(UVec3::new(12, 12, 0)),
        GridPos(UVec3::new(1, 1, 0)),
        Blocking, // Insert the Blocking component if using collision and this entity should block.
    ))
    .spawn((
        Name::new("Npc"),
        // new_astar will default to using A*.
        Pathfind::new_astar(UVec3::new(14, 14, 0)),
        GridPos(UVec3::new(2, 2, 0)),
        Blocking,
    ));
```

## NextPos
The `pathfind` system looks for any entity with a `Changed` `Pathfind` component. It then runs the pathfinding algorithm and when a valid path is found it will insert the next position in the path as a `NextPos` component.

Consume the `NextPos` component by handling the movement and then remove the `NextPos` component. The `next_position` system will insert a new `NextPos` component in a later frame. 

If collision is enabled the `next_position` system will also handle local collision avoidance and adjust the entities path if there is a blocking entity in the path in the `avoidance_distance` look ahead set in `GridSettings`.

Example movement system:
```rust,no_run
fn movement(
    mut commands: Commands,
    mut query: Query<(Entity, &mut GridPos, &NextPos)>,
) {
    for (entity, mut grid_pos, next_pos) in query.iter_mut() {
        // Set the entities GridPos to the NextPos UVec3.
        grid_pos.0 = next_pos.0;

        // Update the entities translation
        let translation = Vec3::new(
            next.0.x as f32 * 32.0, // Assuming tiles are 32x32
            next.0.y as f32 * 32.0,
            0.0
        );

        commands.entity(entity)
            .insert(Transform::from_translation(translation))
            .remove::<NextPos>();
    }
}
```

## Pathfinding/Collision Marker Components

`PathfindingFailed` will inserted into the entity if a path is not found for the desired goal. You can create your own system to handle these failures, or let the built-in system attempt another pathfind for the goal on the next frame.

`AvoidanceFailed` will be inserted when collision is enabled and the entity is not able to path around a local blocking entity. The `reroute_path` system will attempt to find a new full HPA* path in an attempt to resolve the issue. You can handle these in your own custom system if desired.

`RerouteFailed` is added to the component when all attempts to resolve collision pathing issues have failed and means that there's no viable path at all to the entities desired goal. At this point you will need to handle the issue in your own custom system. Whether this is looking for a new goal or waiting a set amount of time before attempting pathing is up to you. Remove the `RerouteFailed` component from the entity when the entity is ready to attempt pathfinding again.

```rust,no_run
fn handle_reroute_failed(
    mut commands: Commands,
    mut query: Query<(Entity, &Pathfind), With<&RerouteFailed>>,
) {
    for (entity, pathfind) {
        let some_new_goal = UVec3::new(3, 30, 0); // Just some random new goal
        commands
            .entity(entity)
            .insert(Pathfind::new(some_new_goal)
            .remove::<RerouteFailed>();
    }
}
```

# Manual Pathfinding

You don't need to use the pathfinding systems in the `NorthstarPlugin` in order to take advantage of this crate. 

You can use both, or choose to not add the `NorthstarPlugin` and call the pathfinding functions completely manually.

If you don't use `NorthstarPlugin` you'll need to maintain your own `BlockingMap` or `HashMap<UVec3, Entity>` to pass to the `pathfind` function to provide it a list of blocked positions.

All of the pathfinding calls can be done on the `Grid` component.

```rust,no_run
fn manual_pathfind(
    mut commands: Commands,
    player: Single<(Entity, &GridPos, &MoveAction), With<Player>>,
    grid: Single<&CardinalGrid>,
    // If using collision you can use the BlockingMap resource to track blockers.
    blocking: Res<BlockingMap>,
) {
    let grid = grid.into_inner();
    let (player, grid_pos, move_action) = player.into_inner();

    let path = grid.pathfind(grid_pos.0, move_action.0, blocking, true);

    // Setting use_partial to true will allow the pathfinding to return a partial path if a complete path isn't found.

    // If you're not using collision you can pass an empty hashmap for the blocking map.
    let path = grid.pathfind(grid_pos.0, move_action.0, HashMap::new(), true);
}
```