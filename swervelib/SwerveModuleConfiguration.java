package swervelib;

public class SwerveModuleConfiguration {
    public final int steerMotorCanId;
    public final int driveMotorCanId;
    public final int absoluteEncoderCanId;
    public final DiscreetAngle absoluteEncoderAngleOffset;
    
    public SwerveModuleConfiguration(int driveMotorCanId, int steerMotorCanId, int absoluteEncoderCanId, DiscreetAngle absoluteEncoderAngleOffset) {
        this.steerMotorCanId = steerMotorCanId;
        this.driveMotorCanId = driveMotorCanId;
        this.absoluteEncoderCanId = absoluteEncoderCanId;
        this.absoluteEncoderAngleOffset = absoluteEncoderAngleOffset;
    }
}
