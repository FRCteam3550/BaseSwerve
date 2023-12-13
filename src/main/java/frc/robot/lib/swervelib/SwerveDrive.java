package frc.robot.lib.swervelib;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.swervelib.ctre.CANCoderAbsoluteEncoder;
import frc.robot.lib.swervelib.ctre.CANCoderAbsoluteEncoderConfiguration;
import frc.robot.lib.swervelib.ctre.TalonFXDriveConfiguration;
import frc.robot.lib.swervelib.ctre.TalonFXDriveController;
import frc.robot.lib.swervelib.ctre.TalonFXSteerConfiguration;
import frc.robot.lib.swervelib.ctre.TalonFXSteerController;
import frc.robot.lib.swervelib.rev.SparkMaxAbsoluteEncoder;
import frc.robot.lib.swervelib.rev.SparkMaxAbsoluteEncoderConfiguration;
import frc.robot.lib.swervelib.rev.SparkMaxDriveConfiguration;
import frc.robot.lib.swervelib.rev.SparkMaxDriveController;
import frc.robot.lib.swervelib.rev.SparkMaxSteerConfiguration;
import frc.robot.lib.swervelib.rev.SparkMaxSteerController;

public class SwerveDrive {
    private final SwerveModule[] modules;

    private static final ChassisSpeeds STOP_SPEEDS = new ChassisSpeeds(0, 0, 0);
    private static final SwerveModuleState ZERO_STATE = new SwerveModuleState();
    private final SwerveModuleState[] stopStates;
    private final SwerveDrivePoseEstimator odometry;
    private static final Pose2d INITIAL_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0)); 

    private final SwerveDriveConfiguration configuration;

    public SwerveDrive(
            SwerveModuleConfiguration frontLeft,
            SwerveModuleConfiguration frontRight,
            SwerveModuleConfiguration backLeft,
            SwerveModuleConfiguration backRight,
            GearRatio gearRatio,
            DriveConfiguration driveConfiguration,
            SteerConfiguration steerConfiguration,
            AbsoluteEncoderConfiguration absoluteEncoderConfiguration,
            SwerveDriveConfiguration swerveDriveConfiguration) {
        this.configuration = swerveDriveConfiguration;

        stopStates = swerveDriveConfiguration.kinematics.toSwerveModuleStates(STOP_SPEEDS);
        modules = new SwerveModule[]{
            createSwerveModule(frontLeft, gearRatio, driveConfiguration, steerConfiguration, absoluteEncoderConfiguration),
            createSwerveModule(frontRight, gearRatio, driveConfiguration, steerConfiguration, absoluteEncoderConfiguration),
            createSwerveModule(backLeft, gearRatio, driveConfiguration, steerConfiguration, absoluteEncoderConfiguration),
            createSwerveModule(backRight, gearRatio, driveConfiguration, steerConfiguration, absoluteEncoderConfiguration)
        };

        odometry = new SwerveDrivePoseEstimator(
            swerveDriveConfiguration.kinematics, 
            swerveDriveConfiguration.gyroAngleSupplier.get(), 
            getModulePositions(), 
            INITIAL_POSE);
    }

    private SwerveModule createSwerveModule(
            SwerveModuleConfiguration moduleConfiguration,
            GearRatio gearRatio,
            DriveConfiguration driveConfiguration,
            SteerConfiguration steerConfiguration,
            AbsoluteEncoderConfiguration absoluteEncoderConfiguration) {

        DriveController driveController;
        SteerController steerController;
        AbsoluteEncoder absoluteEncoder;

        if (driveConfiguration instanceof TalonFXDriveConfiguration) {
            driveController = new TalonFXDriveController(moduleConfiguration.driveMotorCanId, (TalonFXDriveConfiguration)driveConfiguration, gearRatio, configuration.maxSpeedMS);
        }
        else if (driveConfiguration instanceof SparkMaxDriveConfiguration) {
            driveController = new SparkMaxDriveController(moduleConfiguration.driveMotorCanId, (SparkMaxDriveConfiguration)driveConfiguration, gearRatio, configuration.maxSpeedMS);
        }
        else {
            throw new IllegalArgumentException("The method does not support the given driveConfiguration class. We only support Neo and Falcon500 motors.");
        }

        if (absoluteEncoderConfiguration instanceof CANCoderAbsoluteEncoderConfiguration) {
            absoluteEncoder = new CANCoderAbsoluteEncoder(moduleConfiguration.absoluteEncoderCanId, moduleConfiguration.absoluteEncoderAngleOffset, (CANCoderAbsoluteEncoderConfiguration)absoluteEncoderConfiguration);
        }
        else if (absoluteEncoderConfiguration instanceof SparkMaxAbsoluteEncoderConfiguration) {
            absoluteEncoder = new SparkMaxAbsoluteEncoder(moduleConfiguration.absoluteEncoderCanId, moduleConfiguration.absoluteEncoderAngleOffset, (SparkMaxAbsoluteEncoderConfiguration)absoluteEncoderConfiguration);
        }
        else {
            throw new IllegalArgumentException("The method does not support the given absoluteEncoderConfiguration class. We only support CANCoderAbsoluteEncoderConfiguration and SparkMaxAbsoluteEncoderConfiguration.");
        }

        if (steerConfiguration instanceof TalonFXSteerConfiguration) {
            steerController = new TalonFXSteerController(moduleConfiguration.steerMotorCanId, (TalonFXSteerConfiguration)steerConfiguration, gearRatio, absoluteEncoder);
        }
        else if (steerConfiguration instanceof SparkMaxSteerConfiguration) {
            steerController = new SparkMaxSteerController(moduleConfiguration.steerMotorCanId, (SparkMaxSteerConfiguration)steerConfiguration, gearRatio, absoluteEncoder);
        }
        else {
            throw new IllegalArgumentException("The method does not support the given steerConfiguration class. We only support Neo and Falcon500 motors.");
        }
        SwerveModule result = new SwerveModule(driveController, steerController);
        return result;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] result = new SwerveModulePosition[modules.length];

        for (int i = 0; i < result.length; i++) {
            result[i] = new SwerveModulePosition(
                    modules[i].getDrivePositionM(),
                    Rotation2d.fromDegrees(modules[i].getSteerAngle().degrees()));
        }

        return result;
    }

    public Pose2d getEstimatedPositionM() {
        return odometry.getEstimatedPosition();
    }

    public void resetEstimatedPosition(Pose2d actualPoseM) {
        odometry.resetPosition(configuration.gyroAngleSupplier.get(), getModulePositions(), actualPoseM);
    }

    public void setOpenLoopModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setOpenLoopSpeed(
                    states[i].speedMetersPerSecond / configuration.maxSpeedMS,
                    DiscreetAngle.fromRotation(states[i].angle));
        }
    }

    public void stop() {
        setOpenLoopModuleStates(stopStates);
    }

    public void setOpenLoopSpeed(ChassisSpeeds chassisSpeeds) {
        var states = configuration.kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, configuration.maxSpeedMS);
        // for(var state: states) {
        //     state.speedMetersPerSecond = 0;
        // }
        setOpenLoopModuleStates(states);
    }

    /* Useful for calibrating pivot PID. You can use this in a command which is orienting the module at some angle. */
    public void setAngleTo(Rotation2d angle) {
        setOpenLoopModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(0, angle),
            new SwerveModuleState(0, angle),
            new SwerveModuleState(0, angle),
            new SwerveModuleState(0, angle)
        });
    }

    /**
     * Should only be used by WPILib components which are requiring this low level access, like SwerveControllerCommand.
     * For your programmed auto routine, use setClosedLoopSpeed() instead.
     */
    public void setClosedLoopModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setClosedLoopSpeed(
                    states[i].speedMetersPerSecond,
                    DiscreetAngle.fromRotation(states[i].angle));
        }
    }

    public void setClosedLoopSpeed(ChassisSpeeds chassisSpeeds) {
        var states = configuration.kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, configuration.maxSpeedMS);
        setClosedLoopModuleStates(states);
    }

    public void periodic() {
        odometry.update(configuration.gyroAngleSupplier.get(), getModulePositions());
    }

    public DiscreetAngle getSteerAbsoluteAngle(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getSteerAbsoluteAngle();
    } 

    public Double getDriveSpeedMS(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getDriveSpeedMS();
    }

    /**
     * Get the distance travelled by the given module in the encoder's native units.
     * SparkMax: motor rotations. TalonFX: ticks (1/2048 rotation).
     */
    public double getDrivePositionNativeUnits(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getDrivePositionNativeUnits();
    }

    public double getDrivePositionM(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getDrivePositionM();
    }

    public ContinuousAngle getSteerAngle(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getSteerAngle();
    } 

    public double getOpenLoopReferenceSpeedPct(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getOpenLoopReferenceSpeedPct();
    }

    public double getClosedLoopReferenceSpeedMS(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getClosedLoopReferenceSpeedMS();
    }

    public ContinuousAngle getSteerReferenceAngle(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getSteerReferenceAngle();
    }
}
