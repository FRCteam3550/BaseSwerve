package frc.robot.lib.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.swervelib.DriveController;
import frc.robot.lib.swervelib.GearRatio;

public final class TalonFXDriveController implements DriveController {
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
    private final TalonFX motor;
    private final double sensorVelocityCoefficient;
    private final double metersPerTicks;
    private final double ticksPerMeter = 55715;
    private final int moduleID;
    private double referenceSpeedMS = 0;

    public TalonFXDriveController(int motorCanId, Falcon500DriveConfiguration configuration, GearRatio gearRatio, double maxSpeedMS) {
        moduleID = motorCanId;
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        if(configuration.ticksPerMeter != Double.NaN){
            metersPerTicks = 1 / ticksPerMeter;
        } else {
            metersPerTicks = gearRatio.wheelCircumferenceM * gearRatio.driveReduction / TICKS_PER_ROTATION;
        }
        SmartDashboard.putNumber("Meters per ticks", metersPerTicks);
        sensorVelocityCoefficient = metersPerTicks * 10.0;

        if (configuration.hasPidConstants()) {
            if (Double.isNaN(configuration.feedForwardConstant)) {
                var maxTicksPerSeconds = maxSpeedMS / metersPerTicks;
                var maxTicksPer100ms = maxTicksPerSeconds / 10;
                motorConfiguration.slot0.kF = 1023 / maxTicksPer100ms;
            } else {
                motorConfiguration.slot0.kF = configuration.feedForwardConstant;
            }
            motorConfiguration.slot0.kP = configuration.proportionalConstant;
            motorConfiguration.slot0.kI = configuration.integralConstant;
            motorConfiguration.slot0.kD = configuration.derivativeConstant;
        }

        if (configuration.hasVoltageCompensation()) {
            motorConfiguration.voltageCompSaturation = configuration.nominalVoltage;
        }

        if (configuration.hasCurrentLimit()) {
            motorConfiguration.supplyCurrLimit.currentLimit = configuration.currentLimit;
            motorConfiguration.supplyCurrLimit.enable = true;
        }

        motor = new TalonFX(motorCanId);
        motor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS);

        if (configuration.hasVoltageCompensation()) {
            // Enable voltage compensation
            motor.enableVoltageCompensation(true);
        }

        motor.setNeutralMode(NeutralMode.Brake);

        motor.setInverted(gearRatio.driveInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS);
        motor.setSensorPhase(true);

        // Reduce CAN status frame rates
        motor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                STATUS_FRAME_GENERAL_PERIOD_MS,
                CAN_TIMEOUT_MS
        );
    }

    @Override
    public void setOpenLoopSpeed(double pct) {
        motor.set(TalonFXControlMode.PercentOutput, pct);
    }

    @Override
    public void setClosedLoopSpeed(double speedMS) {
        motor.set(TalonFXControlMode.Velocity, speedMS / sensorVelocityCoefficient);
        referenceSpeedMS = speedMS;
    }

    @Override
    public double getSpeedMS() {
        return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
    }

    @Override
    public double getPositionM() {
        SmartDashboard.putNumber("Ticks" + this.moduleID, motor.getSelectedSensorPosition());
        return motor.getSelectedSensorPosition() * metersPerTicks;
    }

    @Override
    public double getPositionTicks() {
        return motor.getSelectedSensorPosition();
    }

    @Override
    public double getReferenceSpeedMS() {
        return referenceSpeedMS; 
    }
}
