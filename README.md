# relic-recovery
Stuy Fission 310<br>
First Tech Challenge Relic Recovery '17-'18 Robot Code<br>
Written and maintained by Ryan Siu<br>
With help from our programmers, Joe Suzuki and Matthew Chan

## Instructions
Clone this repository into the TeamCode directory of the FTC app, which can be found [here](https://github.com/ftctechnh/ftc_app).

## Documentation
Documentation can be found in the `docs` directory.

## Naming Conventions

### Autonomous
All autonomous OpMode Java files should be named as such
```
Auton<Alliance><Target1><Target2><Target...>.java
```

#### Alliance 
- one of Blue, Red, or Both

#### Target
- any of Jewel, Pattern, Glyph, Box (short for cryptobox), Safe (short for safe zone)
- file should be named in the order the targets are reached

### Teleop
The main teleop file should be named `TeleopMain.java`. Any other variations of the main teleop should be named `Teleop<Description>.java`.

### Sensors/Concepts
Sensor or concept OpModes should be named `<Concept/Sensor><Description>.java` and placed in the `prototype` directory.

### Testing/Prototyping
ALL test or prototyping OpMode files should be named `Test<Description>.java` and placed in the `prototype` directory.

### Hardware
Hardware files should be placed in the `hardware` directory. The main hardware map should be named `HardwareMain.java`.
