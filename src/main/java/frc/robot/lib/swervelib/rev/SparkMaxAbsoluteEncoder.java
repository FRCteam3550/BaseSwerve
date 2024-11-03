package frc.robot.lib.swervelib.rev;

import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.lib.SparkMaxUtils;
import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.DiscreetAngle;

public class SparkMaxAbsoluteEncoder implements AbsoluteEncoder {

    private final com.revrobotics.AbsoluteEncoder encoder;

    public SparkMaxAbsoluteEncoder(int motorCanId, DiscreetAngle alignAngle, SparkMaxAbsoluteEncoderConfiguration configuration) {
        var controller = SparkMaxUtils.getController(motorCanId);
        encoder = controller.getAbsoluteEncoder(Type.kDutyCycle);
        throwIfError(encoder.setPositionConversionFactor(360));
        throwIfError(encoder.setZeroOffset(alignAngle.degrees()));
        throwIfError(encoder.setInverted(configuration.inverted));
    }

    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return DiscreetAngle.fromDegrees(encoder.getPosition());
    }

    private void throwIfError(REVLibError error) {
        if (error != REVLibError.kOk) {
            throw new RuntimeException(String.format("Error: %s", error.name()));
        }
    }
}
