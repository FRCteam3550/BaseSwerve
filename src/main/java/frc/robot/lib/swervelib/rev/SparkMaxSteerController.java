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
import com.revrobotics.REVLibError;

public final class SparkMaxSteerController implements SteerController {
    private final CANSparkMax motor;
    private final SparkMaxPIDController controller;
    private final RelativeEncoder motorEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    private ContinuousAngle referenceAngle = ContinuousAngle.fromDegrees(0);

    public SparkMaxSteerController(int motorCanId, SparkMaxSteerConfiguration steerConfiguration, GearRatio gearRatio, AbsoluteEncoder absoluteEncoder) {
        this.absoluteEncoder = absoluteEncoder;

        motor = SparkMaxControllers.getController(motorCanId);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.setInverted(gearRatio.steerInverted);

        if (steerConfiguration.hasVoltageCompensation()) {
            motor.enableVoltageCompensation(steerConfiguration.nominalVoltage);
        }
        if (steerConfiguration.hasCurrentLimit()) {
            motor.setSmartCurrentLimit((int) Math.round(steerConfiguration.currentLimit));
        }

        motorEncoder = motor.getEncoder();
        motorEncoder.setPositionConversionFactor(360 * gearRatio.steerReduction);
        motorEncoder.setVelocityConversionFactor(360 * gearRatio.steerReduction / 60.0);
        try {
            // Wait for the conversion factor to sink in.
            Thread.sleep(200);
        }
        catch(InterruptedException ie) { /* Do nothing */}
        throwIfError(motorEncoder.setPosition(absoluteEncoder.getAbsoluteAngle().degrees()));


        controller = motor.getPIDController();
        if (steerConfiguration.hasPidConstants()) {
            controller.setP(steerConfiguration.proportionalConstant);
            controller.setI(steerConfiguration.integralConstant);
            controller.setD(steerConfiguration.derivativeConstant);
        }
        controller.setFeedbackDevice(motorEncoder);
        motor.burnFlash();
    }

    @Override
    public ContinuousAngle getReferenceAngle() {
        return referenceAngle;
    }

    @Override
    public void setReferenceAngle(ContinuousAngle referenceAngle) {
        this.referenceAngle = referenceAngle;
        controller.setReference(referenceAngle.degrees(), ControlType.kPosition);
    }

    @Override
    public ContinuousAngle getAngle() {
        return ContinuousAngle.fromDegrees(motorEncoder.getPosition());
    }

    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return absoluteEncoder.getAbsoluteAngle();
    }

    private void throwIfError(REVLibError error) {
        if (error != REVLibError.kOk) {
            throw new RuntimeException(String.format("Error: %s", error.name()));
        }
    }
}
