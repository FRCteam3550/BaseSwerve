package frc.robot.lib.swervelib.rev;

import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.lib.MathUtils;
import frc.robot.lib.SparkMaxUtils;
import frc.robot.lib.SystemUtils;
import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.DiscreetAngle;

public class SparkMaxAbsoluteEncoder implements AbsoluteEncoder {
    private static final long SETTING_TIMEOUT_MS = 200;
    private static final double ROT_TO_DEGREES = 360.0;
    private static final double DEGREES_TO_ROT = 1 / ROT_TO_DEGREES;
    private final com.revrobotics.AbsoluteEncoder encoder;

    public SparkMaxAbsoluteEncoder(int motorCanId, DiscreetAngle alignAngle, SparkMaxAbsoluteEncoderConfiguration configuration) {
        var controller = SparkMaxUtils.getController(motorCanId);
        encoder = controller.getAbsoluteEncoder(Type.kDutyCycle);
        SparkMaxUtils.throwIfError(encoder.setPositionConversionFactor(1.0));
        SystemUtils.waitUntil(
            SETTING_TIMEOUT_MS,
            () -> MathUtils.areApproxEqual(1.0, encoder.getPositionConversionFactor(), 0.01)
        );
        var alignAngleRots = alignAngle.degrees() * DEGREES_TO_ROT;
        SparkMaxUtils.throwIfError(encoder.setZeroOffset(alignAngleRots));
        SystemUtils.waitUntil(
            SETTING_TIMEOUT_MS,
            () -> MathUtils.areApproxEqual(alignAngleRots, encoder.getPosition(), 1.0 * DEGREES_TO_ROT)
        );
        SparkMaxUtils.throwIfError(encoder.setInverted(configuration.inverted));
    }

    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return DiscreetAngle.fromDegrees(encoder.getPosition() * ROT_TO_DEGREES);
    }
}
