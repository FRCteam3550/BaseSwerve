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
import frc.robot.lib.swervelib.*;
import frc.robot.lib.swervelib.ctre.CANCoderAbsoluteEncoderConfiguration;
import frc.robot.lib.swervelib.ctre.TalonFXDriveConfiguration;
import frc.robot.lib.swervelib.ctre.TalonFXSteerConfiguration;

import com.kauailabs.navx.frc.AHRS;

public class KrakenMk4iDrivetrain extends SubsystemBase {
    private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 141;
    private static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 142;
    private static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 140;
    private static final DiscreetAngle FRONT_LEFT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(0);

    private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 111;
    private static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 112;
    private static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 110;
    private static final DiscreetAngle FRONT_RIGHT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(0);

    private static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 131;
    private static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 132;
    private static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 130;
    private static final DiscreetAngle BACK_LEFT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(0);

    private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 121;
    private static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 122;
    private static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 120 ;
    private static final DiscreetAngle BACK_RIGHT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(0);

    private static final double FRONT_SIDE_M = .632;
    private static final double RIGHT_SIDE_M = .63;
    private static final double BACK_SIDE_M = .631;
    private static final double LEFT_SIDE_M = .635;

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

    private static final double MAX_SPEED_MS = 4.786;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * This is a measure of how fast the robot can rotate in place.
     * Here we calculate the theoretical maximum angular velocity. You can also
     * replace this with a measured amount.
     */
    private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_SPEED_MS
            / Math.hypot(FRONT_SIDE_M, RIGHT_SIDE_M);

    private static final double STEER_POS_P = 12; // .75
    private static final double STEER_POS_I = 0;
    private static final double STEER_POS_D = 0; // 2

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
        SdsGearRatios.MK4i_L3,
        new TalonFXDriveConfiguration(),
        new TalonFXSteerConfiguration()
            .withPidConstants(STEER_POS_P, STEER_POS_I, STEER_POS_D), 
        new CANCoderAbsoluteEncoderConfiguration(),
        new SwerveDriveConfiguration(
            MAX_SPEED_MS, 
            KINEMATICS, 
            () -> getGyroscopeRotation()
        )
    );

    private final CommandXboxController gamepad;

    public KrakenMk4iDrivetrain(CommandXboxController gamepad) {
        this.gamepad = gamepad;
        setDefaultCommand(drive());

        for(var location : ModuleLocation.values()) {
            setModuleTelemetry(location);
        }
    }

    private static final ShuffleboardTab MODULE_TAB = Shuffleboard.getTab("Modules states");

    private void setModuleTelemetry(ModuleLocation module) {
        ShuffleboardLayout layout = MODULE_TAB.getLayout(module.name + " module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(module.index * 2, 0);
            
        layout.addDouble("Drive speed MS", () -> swerveDrive.getDriveSpeedMS(module));
        layout.addDouble("Steer angle", () -> swerveDrive.getSteerAngle(module).degrees());
        layout.addDouble("Steer reference angle", () -> swerveDrive.getSteerReferenceAngle(module).degrees());
        layout.addDouble("Drive position ticks", () -> swerveDrive.getDrivePositionNativeUnits(module));
    }

    /**
     * Retourne l'angle du Gyromètre dans l'intervalle [0, 360]
     */
    private Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(360 - navx.getFusedHeading());
    }

    public Command drive() {
       return run(() -> {
            var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -gamepad.getLeftY() * MAX_SPEED_MS,
                -gamepad.getLeftX() * MAX_SPEED_MS,
                -gamepad.getRightX() * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                getGyroscopeRotation()
            );

            swerveDrive.setOpenLoopSpeed(chassisSpeeds);
        })
        .andThen(() -> swerveDrive.stop());
    }

    public Command steerAllWheelsAtRestTo(Rotation2d angle) {
        return run(() -> swerveDrive.steerAllWheelsAtRestTo(angle));
    }

    @Override
    public void periodic() {
        swerveDrive.periodic();
    }
}
