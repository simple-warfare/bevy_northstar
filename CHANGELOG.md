## v0.2.2
**BREAKING CHANGES**
`GridSettings` has been replaced with a builder `GridSettingsBuilder`.

Wall has been renamed to Solid for clarity.
`Point::wall` has been renamed to `Point::solid`. 

### Features
* Added `GridSettingsBuilder` to make configuring `Grid` cleaner.
* Neighbors are now precomputed to support more complex neighborhood algorithms


## v0.2.0
First official public release