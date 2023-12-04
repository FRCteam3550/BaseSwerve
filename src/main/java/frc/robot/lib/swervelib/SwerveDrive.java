package frc.robot.lib.swervelib;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.lib.swervelib.ctre.CANCoderAbsoluteEncoder;
import frc.robot.lib.swervelib.ctre.Falcon500DriveConfiguration;
import frc.robot.lib.swervelib.ctre.Falcon500SteerConfiguration;
import frc.robot.lib.swervelib.ctre.TalonFXDriveController;
import frc.robot.lib.swervelib.ctre.TalonFXSteerController;
import frc.robot.lib.swervelib.rev.SparkMaxDriveConfiguration;
import frc.robot.lib.swervelib.rev.SparkMaxDriveController;
import frc.robot.lib.swervelib.rev.SparkMaxSteerConfiguration;
import frc.robot.lib.swervelib.rev.SparkMaxSteerController;

public class SwerveDrive {
    private final SwerveModule[] modules;

    private static final ChassisSpeeds STOP_SPEEDS = new ChassisSpeeds(0, 0, 0);
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

        if (driveConfiguration instanceof Falcon500DriveConfiguration) {
            driveController = new TalonFXDriveController(moduleConfiguration.driveMotorCanId, (Falcon500DriveConfiguration)driveConfiguration, gearRatio, configuration.maxSpeedMS);
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
        else {
            throw new IllegalArgumentException("The method does not support the given absoluteEncoderConfiguration class. We only support CANCoderAbsoluteEncoder.");
        }

        if (steerConfiguration instanceof Falcon500SteerConfiguration) {
            steerController = new TalonFXSteerController(moduleConfiguration.steerMotorCanId, (Falcon500SteerConfiguration)steerConfiguration, gearRatio, absoluteEncoder);
        }
        else if (steerConfiguration instanceof SparkMaxSteerConfiguration) {
            steerController = new SparkMaxSteerController(moduleConfiguration.steerMotorCanId, (SparkMaxSteerConfiguration)steerConfiguration, gearRatio);
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

    public void stop() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setOpenLoopSpeed(
                    stopStates[i].speedMetersPerSecond,
                    DiscreetAngle.fromRotation(stopStates[i].angle));
        }
    }

    public void setOpenLoopSpeed(ChassisSpeeds chassisSpeeds) {
        var states = configuration.kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, configuration.maxSpeedMS);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setOpenLoopSpeed(
                    states[i].speedMetersPerSecond / configuration.maxSpeedMS,
                    DiscreetAngle.fromRotation(states[i].angle));
        }
    }

    public void setClosedLoopSpeed(ChassisSpeeds chassisSpeeds) {
        var states = configuration.kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, configuration.maxSpeedMS);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setClosedLoopSpeed(
                    states[i].speedMetersPerSecond,
                    DiscreetAngle.fromRotation(states[i].angle));
        }
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

    public double getDrivePositionTicks(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getDrivePositionTicks();
    }

    public double getDrivePositionM(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getDrivePositionM();
    }

    public ContinuousAngle getSteerAngle(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getSteerAngle();
    } 

    public double getReferenceSpeedMS(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getReferenceSpeedMS();
    }

    public ContinuousAngle getSteerReferenceAngle(ModuleLocation moduleLocation) {
        return modules[moduleLocation.index].getSteerReferenceAngle();
    }
}
