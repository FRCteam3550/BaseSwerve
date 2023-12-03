package frc.robot.swervelib;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveDriveConfiguration {
    public final double maxSpeedMS;
    public final SwerveDriveKinematics kinematics;
    public final Supplier<Rotation2d> gyroAngleSupplier;

    public SwerveDriveConfiguration(double maxSpeedMs, SwerveDriveKinematics kinematics, Supplier<Rotation2d> gyroAngleSupplier) {
        this.maxSpeedMS = maxSpeedMs;
        this.kinematics = kinematics;
        this.gyroAngleSupplier = gyroAngleSupplier;
    }
}
