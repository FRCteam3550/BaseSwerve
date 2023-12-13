package frc.robot.lib.swervelib.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.lib.swervelib.DriveController;
import frc.robot.lib.swervelib.GearRatio;

public final class SparkMaxDriveController implements DriveController {
    private final CANSparkMax motor;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder encoder;
    private double openLoopReferenceSpeedPct = 0;
    private double closedLoopReferenceSpeedMS = 0;
    private double metersPerMotorRotation;

    public SparkMaxDriveController(int motorCanId, SparkMaxDriveConfiguration configuration, GearRatio gearRatio, double maxSpeedMS) {
        motor = new CANSparkMax(motorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(gearRatio.driveInverted);

        // Setup voltage compensation
        if (configuration.hasVoltageCompensation()) {
            motor.enableVoltageCompensation(configuration.nominalVoltage);
        }

        if (configuration.hasCurrentLimit()) {
            motor.setSmartCurrentLimit((int)configuration.currentLimit);
        }

        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        // Set neutral mode to brake
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Setup absoluteEncoder
        encoder = motor.getEncoder();
        if (configuration.rotationsPerMeter != Double.NaN) {
            metersPerMotorRotation = 1 / configuration.rotationsPerMeter;
        } else {
            metersPerMotorRotation = gearRatio.wheelCircumferenceM * gearRatio.driveReduction;
        }
        encoder.setPositionConversionFactor(metersPerMotorRotation); // Unit by default: motor rotations.
        encoder.setVelocityConversionFactor(metersPerMotorRotation / 60.0); // Unit by default: motor rotations per minute.

        pidController = motor.getPIDController();

        if (configuration.hasPidConstants()) {
            if (Double.isNaN(configuration.feedForwardConstant)) {
                // FF unit: normalized/native
                // Normalized: -1 to 1
                // Native: RPM (velocity) Rotations (position)
                var maxRotationsPerMinutes = maxSpeedMS / metersPerMotorRotation * 60;
                pidController.setFF(1 / maxRotationsPerMinutes);
            } else {
                pidController.setFF(configuration.feedForwardConstant);
            }
            pidController.setP(configuration.proportionalConstant);
            pidController.setI(configuration.integralConstant);
            pidController.setD(configuration.derivativeConstant);
        }
        motor.burnFlash();
    }

    @Override
    public void setOpenLoopSpeed(double pct) {
        openLoopReferenceSpeedPct = pct;
        motor.set(pct);
    }

    @Override
    public void setClosedLoopSpeed(double speedMS) {
        closedLoopReferenceSpeedMS = speedMS;
        pidController.setReference(speedMS, ControlType.kVelocity);
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
    public double getOpenLoopReferenceSpeedPct() {
        return openLoopReferenceSpeedPct; 
    }

    @Override
    public double getClosedLoopReferenceSpeedMS() {
        return closedLoopReferenceSpeedMS; 
    }
}
