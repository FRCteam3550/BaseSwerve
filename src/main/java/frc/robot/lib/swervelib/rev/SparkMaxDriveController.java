package frc.robot.lib.swervelib.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.lib.SparkMaxUtils;
import frc.robot.lib.swervelib.DriveController;
import frc.robot.lib.swervelib.GearRatio;

public final class SparkMaxDriveController implements DriveController {
    private final SparkMax motor;
    private final SparkClosedLoopController pidController;
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final RelativeEncoder encoder;
    private double referenceSpeedMS = 0;
    private double metersPerMotorRotation;

    public SparkMaxDriveController(int motorCanId, SparkMaxDriveConfiguration configuration, GearRatio gearRatio, double maxSpeedMS) {
        motor = SparkMaxUtils.getController(motorCanId); // Already reset to factory defaults
        config.inverted(true);
        config.smartCurrentLimit(38);

        // Setup voltage compensation
        if (configuration.hasVoltageCompensation()) {
            config.voltageCompensation(configuration.nominalVoltage);
        }

        if (configuration.hasCurrentLimit()) {
            config.smartCurrentLimit((int)configuration.currentLimit);
        }
        config.signals.primaryEncoderPositionPeriodMs(20);
        // Set neutral mode to brake
        config.idleMode(IdleMode.kBrake);

        // Setup absoluteEncoder
        encoder = motor.getEncoder();
        if (!Double.isNaN(configuration.rotationsPerMeter)) {
            metersPerMotorRotation = 1.0 / configuration.rotationsPerMeter;
        } else {
            metersPerMotorRotation = gearRatio.wheelCircumferenceM * gearRatio.driveReduction;
        }
        config.absoluteEncoder.positionConversionFactor(metersPerMotorRotation); // Unit by default: motor rotations.
        config.absoluteEncoder.velocityConversionFactor(metersPerMotorRotation / 60.0); // Unit by default: motor rotations per minute.

        pidController = motor.getClosedLoopController();

        if (configuration.hasPidConstants()) {
            if (Double.isNaN(configuration.feedForwardConstant)) {
                // FF unit: normalized/native
                // Normalized: -1 to 1
                // Native: RPS (velocity) Rotations (position)
                var maxRotationsPerMinutes = maxSpeedMS / metersPerMotorRotation * 60;
                config.closedLoop.velocityFF(1 / maxRotationsPerMinutes);
            } else {
                config.closedLoop.velocityFF(configuration.feedForwardConstant);
            }
            config.closedLoop.pid(
                configuration.proportionalConstant, 
                configuration.integralConstant, 
                configuration.derivativeConstant
            );
        }
        SparkMaxUtils.throwIfError(motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        SparkMaxUtils.throwIfError(motor.clearFaults());
    }

    @Override
    public void setOpenLoopSpeed(double pct) {
        motor.set(pct);
    }

    @Override
    public void setClosedLoopSpeed(double speedMS) {
        referenceSpeedMS = speedMS;
        SparkMaxUtils.throwIfError(pidController.setReference(speedMS, SparkBase.ControlType.kVelocity));
    }

    @Override
    public double getSpeedMS() {
        return encoder.getVelocity();
    }

    @Override
    public double getPositionM() {
        return encoder.getPosition();
    }

    @Override
    public double getPositionNativeUnits() {
        return encoder.getPosition() / metersPerMotorRotation;
    }

    @Override
    public double getOutput() {
        return motor.getAppliedOutput();
    }

    @Override
    public double getReferenceSpeedMS() {
        return referenceSpeedMS; 
    }
}
