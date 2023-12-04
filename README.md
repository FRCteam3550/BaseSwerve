# BaseSwerve
​
**Basic Swerve Code for a Swerve Module using Falcon, Neo, or Kraken Motors, a CTRE CANCoder, and a NAVX Gyro**
This code was designed with Swerve Drive Specialties MK4 style modules in mind, but should be easily adaptable to other styles of modules.
​
## Basic configuration
----
​
### Initial configuration in your code
​
The following things must be adjusted to your robot and module's specific parameters. All distance units must be in meters, time units in seconds. Here is the minimal configuration:
​
```java
    private final SwerveDrive swerveDrive = new SwerveDrive(
        new SwerveModuleConfiguration( 
            FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
            FRONT_LEFT_MODULE_STEER_MOTOR_ID, 
            FRONT_LEFT_MODULE_ABSOLUTE_ENCODER_ID, 
            FRONT_LEFT_MODULE_STEER_ALIGN_ANGLE
        ), 
        new SwerveModuleConfiguration( 
            FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID, 
            FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
            FRONT_RIGHT_MODULE_ABSOLUTE_ENCODER_ID, 
            FRONT_RIGHT_MODULE_STEER_ALIGN_ANGLE
        ), 
        new SwerveModuleConfiguration( 
            BACK_LEFT_MODULE_DRIVE_MOTOR_ID,
            BACK_LEFT_MODULE_STEER_MOTOR_ID, 
            BACK_LEFT_MODULE_ABSOLUTE_ENCODER_ID,
            BACK_LEFT_MODULE_STEER_ALIGN_ANGLE
        ), 
        new SwerveModuleConfiguration(
            BACK_RIGHT_MODULE_DRIVE_MOTOR_ID,
            BACK_RIGHT_MODULE_STEER_MOTOR_ID,  
            BACK_RIGHT_MODULE_ABSOLUTE_ENCODER_ID, 
            BACK_RIGHT_MODULE_STEER_ALIGN_ANGLE
        ), 
        SdsGearRatios.MK4_L1, 
        new TalonFXDriveConfiguration(), 
        new TalonFXSteerConfiguration()
            .withPidConstants(STEERPOS_P, STEERPOS_I, STEERPOS_D), 
        new CANCoderAbsoluteEncoderConfiguration(),
        new SwerveDriveConfiguration(
            MAX_SPEED_MS, 
            KINEMATICS, 
            () -> getGyroscopeRotation()
        )
    );
```
​
1. Each of your module's component CAN IDs and initial angle. Set initial angle to 0 to start with, and then follow below instructions for measuring initial angles.
2. Gear ratios and wheel circumference. Constants for Swerve Drive Specialties MK4 L1 to L4 are already given. For other setups, create a `new GearRatio()` with approriate values. This is used for steer control and odometry.
3. Type of drive motor controller and encoder in each module (`TalonFXDriveConfiguration` and `SparkMaxDriveConfiguration` are built in, you can create others).
4. Type of steer motor controller and encoder in each module (`TalonFXSteerConfiguration` and `SparkMaxSteerConfiguration` are built in, you can create others), and its PID constants.
5. Type of absolute encoder in each module (`CANCoderAbsoluteEncoderConfiguration` is built in, you can create others).
6. Finally, a couple of settings related to the entire drivetrain:
    * max speed in m/s. Set to 10 m/s to start with, and follow below instruction to measure it. This is used for desaturation.
    * the `SwerveDriveKinematics`.
    * a method to access the gyro's current angle, as a `Rotation2D`.
​
Then, make sure to setup telemetry, and in particular, make sure to display, for each module:
​
* The absolute angle, through `swerveDrive.getSteerAbsoluteAngle(module)`.
* The steer encoder's angle, as well as its PID set point, through `swerveDrive.getSteerAngle(module)` and `swerveDrive.getSteerReferenceAngle(module)`.
* The drive wheel speed, through `swerveDrive.getDriveSpeedMS(module)`
​
### Measuring and adjusting parameters
​
#### 1 - Initial angles
​
Align all wheels on the physical robot, and observe, on the ShufleBoard or your favorite dashboard, the absolute angle on each module. Set them accordingly in each `SwerveModuleConfiguration()`.
​
#### 2 - Steering PID
​
* Display one module's steer angle and its set point in a graph (in the ShuffleBoard or Glass).
* Code (and bind to buttons) one command setting the angle at 0 degree, and another at 45 degree.
* Fire the robot, and switch alternatively each of the 2 commands. Adjust your PID using module's response.
​
#### 3 - Wheel direction
​
At this point, your drivetrain should be driveable. You may have some wheels going into the wrong direction at startup. If so, add 180 degrees to the initial angle for this module.
​
#### 4 - Max speed
​
Finally, you can now measure the max speed and replace its value in the code, so desaturation works properly.
​
## Advanced configuration
----
​
### More precise odometry
​
You have 2 ways to improve odometry precision:
​
1. The fastest is to measure the distance travelled by your robot with one rotation of the wheel. You can then override the theoretical wheel circumference on the gear ratio:
​
```java
SdsGearRatios.MK4_L1.withWheelCircumference(10.101)
```
​
2. For even more precision, measure directly the ratio from drive encoder native units to meters. For this:
    1. Start by displaying the steer motor position in the ShuffleBoard through `swerveDrive.getDrivePositionNativeUnits(module)`
    2. Place the robot at one end of your test field. Note precisely where the middle of the back of the robot is on that field.
    3. Note the current position of each of the module in their native units, looking at the ShufleBoard. Make the average of this, we'll call it _position1_.
    4. Drive the robot to the other end of that field. Measure which distance (in meters) the middle of the back of the robot travelled from the point noted in 2.1).
    5. Note the current position of each of the module in their native units, looking at the ShufleBoard. Make the average of this, we'll call it _position2_.
    6. The ratio is `(position2 - position1) / distance`. Set it through the drive configuration. For example: `new TalonFXDriveConfiguration().withTicksPerMeter(53600.294)`.
​
### Closed loop drive speed
​
You can set the speed PID settings through the drive configuration. Ex:
​
```java
new TalonFXDriveConfiguration().withPIDConstants(DRIVESPEED_P, DRIVESPEED_I, DRIVESPEED_D)
```
​
Note that in this case, a decent feed forward gain will be computed for you from the max speed parameter. You can override it to the value of your choice with:
​
```java
new TalonFXDriveConfiguration().withPIDConstants(DRIVESPEED_FF, DRIVESPEED_P, DRIVESPEED_I, DRIVESPEED_D)
```
​
## Credits
----
​
The code has been adapted by team 3550 from [SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained](https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained).
​
The readme has been heavily inspired by [Team364/BaseFalconSwerve](https://github.com/Team364/BaseFalconSwerve).
