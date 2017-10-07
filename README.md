# sturdy-barnacle
Stuy Fission 310<br>
First Tech Challenge Relic Recovery '17-'18 Robot Code<br>
Written and maintained by Ryan Siu<br>
With help from our programmers, Joe Suzuki and Matthew Chan

## File Index

| File                    | Contents or Auton Routine                              |
| ----------------------- | ------------------------------------------------------ |
| TestAcquirerPrototype   | OpMode for Saahir's prototype acquirer                 |

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

### Testing/Prototyping
ALL test or prototyping OpMode files should be named ```Test<Description>.java```.
