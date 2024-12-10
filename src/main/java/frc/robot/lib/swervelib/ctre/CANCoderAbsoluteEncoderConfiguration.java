package frc.robot.lib.swervelib.ctre;

import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.AbsoluteEncoderConfiguration;
import frc.robot.lib.swervelib.DiscreetAngle;

public class CANCoderAbsoluteEncoderConfiguration implements AbsoluteEncoderConfiguration{
    public static final int DEFAULT_READING_UPDATE_PERIOD_MS = 100;
    public final int readingUpdatePeriodMs;

    public CANCoderAbsoluteEncoderConfiguration(int readingUpdatePeriodMs) {
        this.readingUpdatePeriodMs = readingUpdatePeriodMs;
    }

    public CANCoderAbsoluteEncoderConfiguration() {
        this(DEFAULT_READING_UPDATE_PERIOD_MS);
    }
    
    public AbsoluteEncoder createAbsoluteEncoder(int encoderCanId, DiscreetAngle alignAngle) {
        return new CANCoderAbsoluteEncoder(encoderCanId, alignAngle, this);
    }
}
