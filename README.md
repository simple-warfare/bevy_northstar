AStar pathfinding crate for Bevy based on Godot's AStar pathfinding. 
Why?: To provide a solution for AStar that doesn't require calculating successors while solving.

Implementation is only partially completed, but supports Sync + Send for Bevy.

Next step is to try to optimize where arc references are required, then implement the rest of the needed functions such as disable_point, remove_point etc.

Current bench is ~13-14 microseconds for solving a 64x64 grid corner to corner compared to ~9-10 microseconds for the Rust Pathfinding crate for the same solution.

