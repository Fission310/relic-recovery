# sturdy-barnacle
Stuy Fission 310<br>
First Tech Challenge Relic Recovery '17-'18 Robot Code<br>
Written and maintained by Ryan Siu<br>
With help from our programmers, Joe Suzuki and Matthew Chan

## File Index

| File                        | Contents or Auton Routine                              |
| --------------------------- | ------------------------------------------------------ |
| ConceptVuMarkIdentification | Test OpMode for sensing VuMarks                        |
| ConceptVuforiaFollowTarget  | Autonomous OpMode that makes a robot follow a target   |
| CraigLauncherTeleop         | Last year's teleop code; retained for demo purposes    |
| HardwareMain                | Contains current robot's hardware map                  |
| SensorBNO055IMU             | Sample OpMode to test REV Expansion Hub IMU            |
| SensorBNO055IMUCalibration  | Sample OpMode to test REV Expansion Hub IMU            |
| TeleopMain                  | Main teleop code                                       |
| TestAcquirerPrototype       | OpMode for Saahir's prototype acquirer                 |
| TestDriveEncodersIMU        | Test OpMode to drive using encoders and IMU            |

**Note:** \* denotes work in progress or incomplete

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
The main teleop file should be named ```TeleopMain.java```. Any other variations of the main teleop should be named ```Teleop<Description>.java```.

### Sensors/Concepts
Sensor or concept OpModes should be named ```<Concept/Sensor><Description>.java```.

### Testing/Prototyping
ALL test or prototyping OpMode files should be named ```Test<Description>.java```.
