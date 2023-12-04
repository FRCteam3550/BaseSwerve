package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Navx;
import frc.robot.lib.gamepad.HolonomicSpeedFilter;
import frc.robot.lib.gamepad.HolonomicSpeedSetPoint;
import frc.robot.lib.swervelib.*;
import frc.robot.lib.swervelib.ctre.Falcon500DriveConfiguration;
import frc.robot.lib.swervelib.ctre.Falcon500SteerConfiguration;
import frc.robot.PilotShuffleboardLayout;

import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

    private static final double DEADBAND = .01;
    private static final double VELOCITY_RANGE_LIMIT = .2;
    
    // valeur 'usine': 0.3152m
    private static final double DISTANCE_BY_WHEEL_ROTATION_M = .315;

    private static final double FRONT_SIDE_M = .415;
    private static final double RIGHT_SIDE_M = .596;
    private static final double BACK_SIDE_M = .420;
    private static final double LEFT_SIDE_M = .594;

    private static final double FRONT_RIGHT_MODULE_X_M = FRONT_SIDE_M / 2;
    private static final double FRONT_RIGHT_MODULE_Y_M = -RIGHT_SIDE_M / 2;

    private static final double FRONT_LEFT_MODULE_X_M = FRONT_SIDE_M / 2;
    private static final double FRONT_LEFT_MODULE_Y_M = LEFT_SIDE_M / 2;

    private static final double BACK_RIGHT_MODULE_X_M = -BACK_SIDE_M / 2;
    private static final double BACK_RIGHT_MODULE_Y_M = -RIGHT_SIDE_M / 2;

    private static final double BACK_LEFT_MODULE_X_M = -BACK_SIDE_M / 2;
    private static final double BACK_LEFT_MODULE_Y_M = LEFT_SIDE_M / 2;

    private static final double DRIVESPEED_P = .4;
    private static final double DRIVESPEED_I = 0;
    private static final double DRIVESPEED_D = 1;

    private static final double STEERPOS_P = .75;
    private static final double STEERPOS_I = 0;
    private static final double STEERPOS_D = 2;

    private static final double TICKS_PER_METER = 53900;

    private static final double MAX_SPEED_MS = 4.786;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_SPEED_MS
            / Math.hypot(FRONT_SIDE_M, RIGHT_SIDE_M);

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            // Front left
            new Translation2d(FRONT_LEFT_MODULE_X_M, FRONT_LEFT_MODULE_Y_M),
            // Front right
            new Translation2d(FRONT_RIGHT_MODULE_X_M, FRONT_RIGHT_MODULE_Y_M),
            // Back left
            new Translation2d(BACK_LEFT_MODULE_X_M, BACK_LEFT_MODULE_Y_M),
            // Back right
            new Translation2d(BACK_RIGHT_MODULE_X_M, BACK_RIGHT_MODULE_Y_M));

    private final ChargedUpField m_field = new ChargedUpField();

    private final AHRS m_navx = Navx.newReadyNavx(); // NavX connected over MXP

    private final SwerveDrive swerveDrive = new SwerveDrive(
        new SwerveModuleConfiguration( 
            FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
            FRONT_LEFT_MODULE_STEER_MOTOR_ID, 
            FRONT_LEFT_MODULE_STEER_ENCODER_ID, 
            FRONT_LEFT_MODULE_STEER_ALIGN_ANGLE
        ), 
        new SwerveModuleConfiguration( 
            FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID, 
            FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
            FRONT_RIGHT_MODULE_STEER_ENCODER_ID, 
            FRONT_RIGHT_MODULE_STEER_ALIGN_ANGLE
        ), 
        new SwerveModuleConfiguration( 
            BACK_LEFT_MODULE_DRIVE_MOTOR_ID,
            BACK_LEFT_MODULE_STEER_MOTOR_ID, 
            BACK_LEFT_MODULE_STEER_ENCODER_ID,
            BACK_LEFT_MODULE_STEER_ALIGN_ANGLE
        ), 
        new SwerveModuleConfiguration(
            BACK_RIGHT_MODULE_DRIVE_MOTOR_ID,
            BACK_RIGHT_MODULE_STEER_MOTOR_ID,  
            BACK_RIGHT_MODULE_STEER_ENCODER_ID, 
            BACK_RIGHT_MODULE_STEER_ALIGN_ANGLE
        ), 
        SdsGearRatios.MK4_L1
            .withWheelCircumference(DISTANCE_BY_WHEEL_ROTATION_M), 
        new Falcon500DriveConfiguration()
            .withPIDConstants(DRIVESPEED_P, DRIVESPEED_I, DRIVESPEED_D)
            .withTicksPerMeter(TICKS_PER_METER), 
        new Falcon500SteerConfiguration()
            .withPidConstants(STEERPOS_P, STEERPOS_I, STEERPOS_D), 
        new CANCoderAbsoluteEncoderConfiguration(),
        new SwerveDriveConfiguration(
            MAX_SPEED_MS, 
            KINEMATICS, 
            () -> getGyroscopeRotation()
        )
    );

    private final HolonomicSpeedFilter m_speedFilter = new HolonomicSpeedFilter()
            .withDeadband(DEADBAND)
            .withRangeLimits(1, VELOCITY_RANGE_LIMIT)
            .withConversionRatios(MAX_SPEED_MS, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
            .withTranslationInputSpeedLimit(1.5);

    private final XboxController m_gamepad;

    public DrivetrainSubsystem(XboxController gamepad) {
        m_gamepad = gamepad;
        setDefaultCommand(drive());

        for(var location : ModuleLocation.values()) {
            setModuleTelemetry(location);
        }
    }

    private void setModuleTelemetry(ModuleLocation module) {
        ShuffleboardLayout layout = PilotShuffleboardLayout.MODULE_TAB.getLayout(module.name + " module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(module.index * 2, 0);
            
        layout.addDouble("Drive speed MS", () -> swerveDrive.getDriveSpeedMS(module));
        layout.addDouble("Drive reference speed MS", () -> swerveDrive.getReferenceSpeedMS(module));
        layout.addDouble("Steer angle", () -> swerveDrive.getSteerAngle(module).degrees());
        layout.addDouble("Steer reference angle", () -> swerveDrive.getSteerReferenceAngle(module).degrees());
        layout.addDouble("Drive position ticks", () -> swerveDrive.getDrivePositionTicks(module));
    }

    /**
     * Retourne l'angle du Gyromètre dans l'intervalle [0, 360]
     */
    private Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(360 - m_navx.getFusedHeading());
    }

    private void stopMotor() {
        swerveDrive.stop();
    }

    public Command drive() {
        var cmd = run(() -> {
            var gamepadSpeeds = new HolonomicSpeedSetPoint(
                    m_gamepad.getLeftX(),
                    -m_gamepad.getLeftY(),
                    -m_gamepad.getRightX());
            var filteredSpeeds = m_speedFilter.filter(gamepadSpeeds);
            var allianceSign = -m_field.gridRobotXDirection();
            var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    filteredSpeeds.ySpeed * allianceSign,
                    -filteredSpeeds.xSpeed * allianceSign,
                    filteredSpeeds.rotationSpeed,
                    swerveDrive.getEstimatedPositionM().getRotation());
            swerveDrive.setOpenLoopSpeed(chassisSpeeds);
        }).andThen(this::stopMotor)
                .withName("drive");
        return cmd;
    }

    public Command stopMotorCommand() {
        return runOnce(this::stopMotor);
    }

    @Override
    public void periodic() {
        swerveDrive.periodic();
    }
}
