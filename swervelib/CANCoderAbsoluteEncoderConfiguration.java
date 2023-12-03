package swervelib;

public class CANCoderAbsoluteEncoderConfiguration implements AbsoluteEncoderConfiguration{
    public static final int DEFAULT_READING_UPDATE_PERIOD_MS = 100;
    public final int readingUpdatePeriodMs;

    public CANCoderAbsoluteEncoderConfiguration(int readingUpdatePeriodMs) {
        this.readingUpdatePeriodMs = readingUpdatePeriodMs;
    }

    public CANCoderAbsoluteEncoderConfiguration() {
        this(DEFAULT_READING_UPDATE_PERIOD_MS);
    }
    
}
