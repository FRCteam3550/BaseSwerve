package frc.robot.swervelib.ctre;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import frc.robot.swervelib.AbsoluteEncoder;
import frc.robot.swervelib.CANCoderAbsoluteEncoderConfiguration;
import frc.robot.swervelib.DiscreetAngle;

public class CANCoderAbsoluteEncoder implements AbsoluteEncoder {
    private static final int TIMEOUT_MS = 250;
    private static final boolean COUNTER_CLOCKWISE = false;
    private final CANCoder encoder;

    public CANCoderAbsoluteEncoder(int CANId, DiscreetAngle alignAngle, CANCoderAbsoluteEncoderConfiguration configuration) {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = -alignAngle.degrees();
        config.sensorDirection = COUNTER_CLOCKWISE;

        encoder = new CANCoder(CANId);
        encoder.configAllSettings(config, TIMEOUT_MS);
        encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, configuration.readingUpdatePeriodMs, TIMEOUT_MS);
    }
    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return DiscreetAngle.fromDegrees(encoder.getAbsolutePosition());
    }

}
