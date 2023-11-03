# godot-vehicle-sandbox

A playground for developing a semi-realistic vehicle simulation for games in the Godot Engine.
The goal of the project is to develop vehicles that behave as close as possible to real vehicles while keeping the code as simple and fast as possible.
Vehicles are to be made up of different configurable building blocks to allow the creation of various types of vehicles, e.g. FWD/RWD/AWD combustion engine/electric cars, trucks, trailers, trikes, bikes, etc.


## Building blocks

Vehicles are created from a set of nodes and resources.
In general, the nodes are supposed to connect the various components, while the resources are supposed to be independent, so as to be potentially reusable in multiple contexts.
For example, `Axle` depends on `Wheel`, but `Transmission` does not depend on `Motor` or vice versa.


### Nodes

- `Vehicle`
    - Root node of your vehicle
    - Must contain at least one `Axle`
    - Is `RigidBody3D` and requires collision shape(s), realistic mass and center of mass
    - Can be assigned a `Motor` to drive the vehicle
    - Can be assigned a `Transmission` to support multiple gears
    - Can be assigned a `Differential` to split torque between front and rear axle
        - Will be ignored if vehicle does not have exactly two axles
- `Axle`
    - Distributes torque input to a set of wheels
    - Must be a child of `Vehicle` and contain at least one `Wheel`
    - Can be assigned a `Differential` to split torque between left and right wheel
        - Will be ignored if axle does not have exactly two wheels
    - Can be assinged an `AntiRollBar` to reduce vehicle roll
- `Wheel`
    - Provides traction when in contact with the ground
    - Must be a child of `Axle`
    - *Must* be assigned a `Tire` for determining traction forces
    - Can be assigned a `Suspension` for independent wheel suspension
        - Consider always using a suspension; wheels without suspension will currently sink into the ground unless prevented in some other way
    - Can be assigned a `Brake` for braking and traction control

`Axle` and `Wheel` could also be used outside of the intended hierarchy, but would require manual handling that is normally provided by their intended parent node.


### Resources

Note that some resources are stateless and single instances can be reused while others are stateful cannot be reused.
Resources that would commonly be reused tend to be stateless, including `Tire`, `Suspension` and `Brake`.
Here, state refers to dynamic input at runtime, e.g. current suspension compression, rather than properties that are considered static configuration, e.g. suspension stiffness.
Configuration parameters can still be changed at runtime and will affect all dependents, e.g. all wheels using a common suspension resource.

- `Motor` (stateful)
    - Generates torque based on throttle input, RPM and a user-defined torque curve
- `Transmission` (stateful)
    - Transforms input torque and RPM based on use-defined gear ratios (manual or automated)
- `Differential` (stateful)
    - Splits input torque between two outputs at a fixed ratio and based on their feedback
- `AntiRollBar` (stateful)
    - Reduces vehicle roll by reducing the difference in suspension compression of an axle's wheels
- `Tire` (stateless)
    - Determines traction forces based on user-defined longitudinal and lateral friction curves
- `Suspension` (stateless)
    - Per-wheel spring-damper suspension keeping the vehicle upright and determining wheel load
- `Brake` (stateless)
    - Defines the maximum torque values for stopping a wheel via normal brake or handbrake


## Example

A typical four-wheeled car should have the following the structure.

```
Vehicle (Motor, Transmission, Differential)
├ Collision Shapes
├ Meshes
├ Axle (Differential, AntiRollBar)
│ ├ Wheel (Tire, Suspension, Brake)
│ │ └ Mesh
│ └ Wheel (Tire, Suspension, Brake)
│   └ Mesh
└ Axle (Differential, AntiRollBar)
  ├ Wheel (Tire, Suspension, Brake)
  │ └ Mesh
  └ Wheel (Tire, Suspension, Brake)
```

For concrete examples see the [/vehicles](vehicles) directory.


## Input

See the `Project Settings > Input Map` for all input mappings, but here is a short summary.

Note that holding brake does *not* automatically reverse.
You need to manually shift into reverse and then accelerate as usual.
Shifting is possible even if the transmission is automated, but gear changes between forward gears will quickly be overriden by the automation algorithm.

### Keyboard

- W/A/S/D or arrow keys - acceleration, brake and steering
- Space - handbrake
- E/Q - shift up/down
- C - change camera

### Game controller

- Triggers - acceleration and brake
- Shoulders - shift up/down
- Left stick - steering
- Bottom action button - handbrake
- Top action button - change camera


## Attribution

The provided example vehicle uses the following third-party assets.

- "Mercedes Benz G-class W263" (https://skfb.ly/6SHrP) by Lexyc16 is licensed under Creative Commons Attribution (http://creativecommons.org/licenses/by/4.0/).
- "Car tire squeal skid loop" (https://opengameart.org/content/car-tire-squeal-skid-loop) by qubodup is licensed under Creative Commons Attribution (CC-BY 3.0) (http://creativecommons.org/licenses/by/3.0/).
- "Motor Loop 2" (https://pixabay.com/sound-effects/motor-loop-2-103532/) by Pixabay is licensed under Pixabay Content License (https://pixabay.com/service/license-summary/).
