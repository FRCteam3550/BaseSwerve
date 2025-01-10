package frc.robot.lib.swervelib.rev;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.lib.SparkMaxUtils;
import frc.robot.lib.SystemUtils;
import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.DiscreetAngle;

public class SparkMaxAbsoluteEncoder implements AbsoluteEncoder {
    private static final long SETTING_TIMEOUT_MS = 500;
    private static final double ROT_TO_DEGREES = 360.0;
    private static final double CONVERSION_RATE = 1.0;
    private static final double DEGREES_TO_ROT = 1 / ROT_TO_DEGREES;
    private final com.revrobotics.AbsoluteEncoder encoder;
    private final double inversionMultiplier;
    private final SparkMaxConfig config = new SparkMaxConfig();

    public SparkMaxAbsoluteEncoder(int motorCanId, DiscreetAngle alignAngle, SparkMaxAbsoluteEncoderConfiguration configuration) {
        inversionMultiplier = configuration.inverted ? -1 : 1;
        final var controller = SparkMaxUtils.getController(motorCanId);
        encoder = controller.getAbsoluteEncoder();
        // encoder.setInverted() is not having any effect. Invert it ourselves.
        // SparkMaxUtils.throwIfError(encoder.setInverted(configuration.inverted));
        config.absoluteEncoder.positionConversionFactor(CONVERSION_RATE);
        // setZeroOffset() does not accept negative values, so to invert the angle, we substract from 1.0 if inverted.
        final var alignAngleRots = configuration.inverted ? 1.0 - alignAngle.rotations() : alignAngle.rotations();
        config.absoluteEncoder.zeroOffset(alignAngleRots);

        controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SystemUtils.waitEqual(
            "setZeroOffset for abs encoder " + motorCanId,
            SETTING_TIMEOUT_MS,
            () -> controller.configAccessor.absoluteEncoder.getZeroOffset(),
            alignAngleRots,
            1.0 * DEGREES_TO_ROT
        );
        SystemUtils.waitEqual(
            "setPositionConversionFactor for abs encoder " + motorCanId,
            SETTING_TIMEOUT_MS,
            () -> controller.configAccessor.absoluteEncoder.getPositionConversionFactor(),
            CONVERSION_RATE, 
            0.01
        );
    }

    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return DiscreetAngle.fromRotations(encoder.getPosition() * inversionMultiplier);
    }
}
