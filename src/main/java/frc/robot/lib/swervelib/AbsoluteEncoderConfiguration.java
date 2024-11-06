package frc.robot.lib.swervelib;

public interface AbsoluteEncoderConfiguration {
    /**
     * Creates an absolute encoder for a swerve module.
     * For non-CAN encoder, for example internal absolute encodre, you can safely ignore the CAN id.
     */
    public AbsoluteEncoder createAbsoluteEncoder(int encoderCanId, DiscreetAngle alignAngle);
}
