package frc.robot.lib.swervelib.rev;

import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.AbsoluteEncoderConfiguration;
import frc.robot.lib.swervelib.DiscreetAngle;

public class SparkMaxAbsoluteEncoderConfiguration implements AbsoluteEncoderConfiguration{
    public static final boolean DEFAULT_INVERTED = false;
    public final boolean inverted;

    public SparkMaxAbsoluteEncoderConfiguration() {
        this(DEFAULT_INVERTED);
    }

    public SparkMaxAbsoluteEncoderConfiguration(boolean inverted) {
        this.inverted = inverted;
    }
   
    public AbsoluteEncoder createAbsoluteEncoder(int encoderCanId, DiscreetAngle alignAngle) {
        return new SparkMaxAbsoluteEncoder(encoderCanId, alignAngle, this);
    }
}
