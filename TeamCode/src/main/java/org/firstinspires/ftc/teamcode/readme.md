# TeamCode Module

This is where we put our OpModes.
### IMPORTANT:
#### SCROLL DOWN FOR NAMING CONVENTIONS. READ THEM AND FOLLOW THEM.

## Creating your own OpModes

There are a few ways you can do this:
#### 1. Copy and customize a sample OpMode: 

Sample opmodes exist in the FtcRobotController module. Open the module, then go to 
`java -> org.firstinspires.ftc.robotcontroller -> external -> samples`
(Expand the tree elements by clicking the little triangle next to them.)

Once you are there, just copy the class you want in there into the TeamCode module, rename it,
and modify it.

#### 2. Create an OpMode yourself

Each TeleOp OpMode must start with 2 lines of code like these:
```
 @TeleOp(name="Name of the TeleOp", group="TeleOps")
 @Disabled
```
And Autonomous OpModes start with these 2 lines:
```
 @Autonomous(name="Name of the Autonomous", group="AutoOps")
 @Disabled
```
This code takes care of registering the OpMode - you can safely ignore the `FtcOpModeRegister` class.

## Naming conventions
**Names are important. Without proper naming conventions, it is very hard to be organized.
Names must be concise, descriptive, and apply to our rules. If they aren't any of those, they will be changed.**
#### Classes
* Every word starts our with a capital letter, and there are no spaces. `AutoButtonPusher` is a good
name, but `Autobuttonpusher` or `Auto Button Pusher` are bad names.
* Name them concisely and descriptively. `TeleOpThatDoesStuff` is not descriptive. But don't be too long - names over
20 characters long will throw an error.
* Proof of Concept OpModes should have names that end in `PoC`.
* TeleOp OpModes should have names that start with `TeleOp`.
* Autonomous OpModes should have names that start with `Auto`.

#### Variables
* Variables **never** start with a capital letter.
* Variables names should be seperated_by_underscores. Hardware devices (such as sensors) should be in camelCase.
* Booleans (true/false variables) should be phrased like statements: `weAreRed` and `opModeIsActive` are
good examples.
* Name them concisely and descriptively. `gamepadLeftJoystickForwardBackwardPower` is waaaaay too long.
`movementPower` is too vague. `leftDrivePower` is good.
