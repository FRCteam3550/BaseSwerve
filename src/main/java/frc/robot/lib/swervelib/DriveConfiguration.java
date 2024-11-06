package frc.robot.lib.swervelib;

public interface DriveConfiguration {
    public DriveController createDriveController(int motorCanId, GearRatio gearRatio, double maxSpeedMS);
}
