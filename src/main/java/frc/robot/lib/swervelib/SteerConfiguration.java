package frc.robot.lib.swervelib;

public interface SteerConfiguration {
    public SteerController createSteerController(int motorCanId, GearRatio gearRatio, AbsoluteEncoder absoluteEncoder);
}
