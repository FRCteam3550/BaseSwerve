package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lib.Navx;
import frc.robot.lib.swervelib.DiscreetAngle;
import frc.robot.lib.swervelib.ModuleLocation;
import frc.robot.lib.swervelib.RevGearRatios;
import frc.robot.lib.swervelib.SwerveDrive;
import frc.robot.lib.swervelib.SwerveDriveConfiguration;
import frc.robot.lib.swervelib.SwerveModuleConfiguration;
import frc.robot.lib.swervelib.rev.SparkMaxAbsoluteEncoderConfiguration;
import frc.robot.lib.swervelib.rev.SparkMaxDriveConfiguration;
import frc.robot.lib.swervelib.rev.SparkMaxSteerConfiguration;

import com.kauailabs.navx.frc.AHRS;

public class MaxSwerveDrivetrain extends SubsystemBase {
    private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 8;
    private static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 6;
    private static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 6;
    private static final DiscreetAngle FRONT_LEFT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(45.25+180); // -90

    private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 7;
    private static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 5;
    private static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 5;
    private static final DiscreetAngle FRONT_RIGHT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(240.5); // 180

    private static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 2;
    private static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 4;
    private static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 4;
    private static final DiscreetAngle BACK_LEFT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(59+180); // 180

    private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 1;
    private static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 3;
    private static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 3;
    private static final DiscreetAngle BACK_RIGHT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(216); // -90

    private static final double FRONT_SIDE_M = .47;
    private static final double RIGHT_SIDE_M = .47;
    private static final double BACK_SIDE_M = .47;
    private static final double LEFT_SIDE_M = .47;

    private static final double FRONT_RIGHT_MODULE_X_M = FRONT_SIDE_M / 2;
    private static final double FRONT_RIGHT_MODULE_Y_M = -RIGHT_SIDE_M / 2;

    private static final double FRONT_LEFT_MODULE_X_M = FRONT_SIDE_M / 2;
    private static final double FRONT_LEFT_MODULE_Y_M = LEFT_SIDE_M / 2;

    private static final double BACK_RIGHT_MODULE_X_M = -BACK_SIDE_M / 2;
    private static final double BACK_RIGHT_MODULE_Y_M = -RIGHT_SIDE_M / 2;

    private static final double BACK_LEFT_MODULE_X_M = -BACK_SIDE_M / 2;
    private static final double BACK_LEFT_MODULE_Y_M = LEFT_SIDE_M / 2;

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(FRONT_LEFT_MODULE_X_M, FRONT_LEFT_MODULE_Y_M),
        new Translation2d(FRONT_RIGHT_MODULE_X_M, FRONT_RIGHT_MODULE_Y_M),
        new Translation2d(BACK_LEFT_MODULE_X_M, BACK_LEFT_MODULE_Y_M),
        new Translation2d(BACK_RIGHT_MODULE_X_M, BACK_RIGHT_MODULE_Y_M)
    );

    private static final double MAX_SPEED_MS = 3.0;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * This is a measure of how fast the robot can rotate in place.
     * Here we calculate the theoretical maximum angular velocity. You can also
     * replace this with a measured amount.
     */
    private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_SPEED_MS
        / Math.hypot(FRONT_SIDE_M, RIGHT_SIDE_M);

    private static final double STEER_POS_P = .08;
    private static final double STEER_POS_I = 0;
    private static final double STEER_POS_D = 0.2;

    private final AHRS navx = Navx.newReadyNavx(); // NavX connected over MXP

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
        RevGearRatios.SWERVE_MAX_3IN_HIGH,
        new SparkMaxDriveConfiguration(),
        new SparkMaxSteerConfiguration()
            .useInternalAbsoluteEncoderForFeedback()
            .withPidGains(STEER_POS_P, STEER_POS_I, STEER_POS_D), 
        new SparkMaxAbsoluteEncoderConfiguration(true),
        new SwerveDriveConfiguration(
            MAX_SPEED_MS, 
            KINEMATICS, 
            () -> getGyroscopeRotation()
        )
    );

    private final CommandXboxController gamepad;

    public MaxSwerveDrivetrain(CommandXboxController gamepad) {
        this.gamepad = gamepad;
        setDefaultCommand(drive());

        for(var location : ModuleLocation.values()) {
            setModuleTelemetry(location);
        }

        ShuffleboardLayout layout = MODULE_TAB.getLayout("Drivetrain", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(8, 0);
        layout.addDouble("Navx angle D", () -> getGyroscopeRotation().getDegrees());
    }

    private static final ShuffleboardTab MODULE_TAB = Shuffleboard.getTab("Modules states");

    private void setModuleTelemetry(ModuleLocation module) {
        ShuffleboardLayout layout = MODULE_TAB.getLayout(module.name + " module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(module.index * 2, 0);
            
        layout.addDouble("Drive speed ms", () -> swerveDrive.getDriveSpeedMS(module));
        layout.addDouble("Steer angle deg", () -> swerveDrive.getSteerAngle(module).degrees());
        layout.addDouble("Steer ref angle deg", () -> swerveDrive.getSteerReferenceAngle(module).degrees());
        layout.addDouble("Drive position rots", () -> swerveDrive.getDrivePositionNativeUnits(module));
        layout.addDouble("Absolute angle", () -> swerveDrive.getSteerAbsoluteAngle(module).degrees());
        layout.addDouble("Diff encoder", () -> swerveDrive.getSteerAbsoluteAngle(module).degrees() - swerveDrive.getSteerAngle(module).degrees());
    }

    /**
     * Retourne l'angle du Gyromètre dans l'intervalle [0, 360]
     */
    private Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(360 - navx.getFusedHeading());
    }

    public Command drive() {
       return run(() -> {
            var chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                gamepad.getLeftX() * 0.2 * MAX_SPEED_MS,
                -gamepad.getLeftY() * 0.2 * MAX_SPEED_MS,
                -gamepad.getRightX() * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                getGyroscopeRotation()
            );

            swerveDrive.setOpenLoopSpeed(chassisSpeed);
        })
        .andThen(() -> swerveDrive.stop());
    }

    public Command steerAllWheelsAtRestTo(Rotation2d angle) {
        return run(() -> swerveDrive.steerAllWheelsAtRestTo(angle))
            .withTimeout(1)
            .andThen(() -> swerveDrive.stop());
    }

    @Override
    public void periodic() {
        swerveDrive.periodic();
    }
}
