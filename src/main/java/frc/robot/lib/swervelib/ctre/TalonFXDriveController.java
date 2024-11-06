package frc.robot.lib.swervelib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.lib.swervelib.DriveController;
import frc.robot.lib.swervelib.GearRatio;

public final class TalonFXDriveController implements DriveController {
    private final TalonFX motor;
    private final double metersPerRotation;
    private double referenceSpeedMS = 0;

    public TalonFXDriveController(int motorCanId, TalonFXDriveConfiguration configuration, GearRatio gearRatio, double maxSpeedMS) {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        if(!Double.isNaN(configuration.rotationsPerMeter)){
            metersPerRotation = 1 / configuration.rotationsPerMeter;
        } else {
            metersPerRotation = gearRatio.wheelCircumferenceM * gearRatio.driveReduction;
        }

        if (configuration.hasPidConstants()) {
            if (Double.isNaN(configuration.feedForwardConstant)) {
                var maxRotationsPerSeconds = maxSpeedMS / metersPerRotation;
                motorConfiguration.Slot0.kV = 12 / maxRotationsPerSeconds;
            } else {
                motorConfiguration.Slot0.kV = configuration.feedForwardConstant;
            }
            motorConfiguration.Slot0.kP = configuration.proportionalConstant;
            motorConfiguration.Slot0.kI = configuration.integralConstant;
            motorConfiguration.Slot0.kD = configuration.derivativeConstant;
        }
 
        if (configuration.hasCurrentLimit()){
                motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.currentLimit;
                motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        }
          
        motor = new TalonFX(motorCanId);

        motorConfiguration.MotorOutput.Inverted = gearRatio.driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        motor.getConfigurator().apply(motorConfiguration);
    }

    @Override
    public void setOpenLoopSpeed(double pct) {
        motor.setVoltage(pct * 12);

    }

    @Override
    public void setClosedLoopSpeed(double speedMS) {
        motor.setControl(new VelocityVoltage(   speedMS / metersPerRotation));
        referenceSpeedMS = speedMS;
    }

    @Override
    public double getSpeedMS() {
        return motor.getVelocity().getValueAsDouble() * metersPerRotation;
    }

    @Override
    public double getPositionM() {
        return motor.getPosition().getValueAsDouble() * metersPerRotation;
    }

    @Override
    public double getPositionNativeUnits() {
        return motor.getPosition().getValueAsDouble();
    }
    
    @Override
    public double getOutput() {
        return motor.getClosedLoopOutput().getValueAsDouble();
    }

    @Override
    public double getReferenceSpeedMS() {
        return referenceSpeedMS; 
    }
}
