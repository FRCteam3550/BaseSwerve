package frc.robot.lib.swervelib.ctre;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.robot.lib.TalonFXUtils;
import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.DiscreetAngle;

public class CANCoderAbsoluteEncoder implements AbsoluteEncoder {
    private static final int TIMEOUT_MS = 250;
    private static final SensorDirectionValue COUNTER_CLOCKWISE = SensorDirectionValue.CounterClockwise_Positive;
    private final CANcoder encoder;

    public CANCoderAbsoluteEncoder(int CANId, DiscreetAngle alignAngle, CANCoderAbsoluteEncoderConfiguration configuration) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config.MagnetSensor.MagnetOffset = -alignAngle.degrees() / 360.0;
        config.MagnetSensor.SensorDirection = COUNTER_CLOCKWISE;

        encoder = new CANcoder(CANId);
        TalonFXUtils.throwIfError(encoder.getConfigurator().apply(config));
        // encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, configuration.readingUpdatePeriodMs, TIMEOUT_MS);
        TalonFXUtils.throwIfError(encoder.getPosition().setUpdateFrequency(TIMEOUT_MS)); //TODO
    }

    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return DiscreetAngle.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
    }
}
