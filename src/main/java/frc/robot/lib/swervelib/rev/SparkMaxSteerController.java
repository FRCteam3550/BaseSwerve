package frc.robot.lib.swervelib.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.ContinuousAngle;
import frc.robot.lib.swervelib.DiscreetAngle;
import frc.robot.lib.swervelib.GearRatio;
import frc.robot.lib.swervelib.SteerController;

import com.revrobotics.CANSparkMaxLowLevel;

public final class SparkMaxSteerController implements SteerController {
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

    private final CANSparkMax motor;
    private final SparkMaxPIDController controller;
    private final RelativeEncoder motorEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    private ContinuousAngle referenceAngle = ContinuousAngle.fromDegrees(0);

    private double resetIteration = 0;

    public SparkMaxSteerController(int motorCanId, SparkMaxSteerConfiguration steerConfiguration, GearRatio gearRatio) {
        absoluteEncoder = steerConfiguration.absoluteEncoder;

        motor = new CANSparkMax(motorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setInverted(!gearRatio.steerInverted);
        if (steerConfiguration.hasVoltageCompensation()) {
            motor.enableVoltageCompensation(steerConfiguration.nominalVoltage);
        }
        if (steerConfiguration.hasCurrentLimit()) {
            motor.setSmartCurrentLimit((int) Math.round(steerConfiguration.currentLimit));
        }

        motorEncoder = motor.getEncoder();
        motorEncoder.setPositionConversionFactor(2.0 * Math.PI * gearRatio.steerReduction);
        motorEncoder.setVelocityConversionFactor(2.0 * Math.PI * gearRatio.steerReduction / 60.0);
        motorEncoder.setPosition(steerConfiguration.absoluteEncoder.getAbsoluteAngle().radians());

        controller = motor.getPIDController();
        if (steerConfiguration.hasPidConstants()) {
            controller.setP(steerConfiguration.proportionalConstant);
            controller.setI(steerConfiguration.integralConstant);
            controller.setD(steerConfiguration.derivativeConstant);
        }
        controller.setFeedbackDevice(motorEncoder);
    }

    @Override
    public ContinuousAngle getReferenceAngle() {
        return referenceAngle;
    }

    @Override
    public void setReferenceAngle(ContinuousAngle referenceAngle) {
        // Reset the NEO's absoluteEncoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute absoluteEncoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (motorEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                double absoluteAngle = absoluteEncoder.getAbsoluteAngle().radians();
                motorEncoder.setPosition(absoluteAngle);
            }
        } else {
            resetIteration = 0;
        }

        this.referenceAngle = referenceAngle;

        controller.setReference(referenceAngle.radians(), ControlType.kPosition);
    }

    @Override
    public ContinuousAngle getAngle() {
        return ContinuousAngle.fromRadians(motorEncoder.getPosition());
    }

    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return absoluteEncoder.getAbsoluteAngle();
    }

}
