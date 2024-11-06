package frc.robot.lib.swervelib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.lib.MathUtils;
import frc.robot.lib.SystemUtils;
import frc.robot.lib.TalonFXUtils;
import frc.robot.lib.swervelib.*;

public final class TalonFXSteerController implements SteerController {
    private static final double CAN_TIMEOUT_S = 0.250;
    private static final long SETTINGS_APPLIED_WAIT_TIMEOUT_MS = 500;

    private final TalonFX motor;
    private final double steerMotorToMechanismReduction;
    private final AbsoluteEncoder absoluteEncoder;
    private final PositionVoltage positionVoltage = new PositionVoltage(0, 0, false, 0, 0, false, false, false);

    private ContinuousAngle referenceAngle = ContinuousAngle.fromDegrees(0.0);

    public TalonFXSteerController(int motorCanId, TalonFXSteerConfiguration steerConfiguration, GearRatio gearRatio, AbsoluteEncoder absoluteEncoder) {
        steerConfiguration.ensureHasPidConstants();
        steerMotorToMechanismReduction = gearRatio.steerMotorToMechanismReduction;
        this.absoluteEncoder = absoluteEncoder;

        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0.kP = steerConfiguration.proportionalConstant;
        motorConfiguration.Slot0.kI = steerConfiguration.integralConstant;
        motorConfiguration.Slot0.kD = steerConfiguration.derivativeConstant;
        if (steerConfiguration.hasCurrentLimit()) {
            motorConfiguration.CurrentLimits.SupplyCurrentLimit = steerConfiguration.currentLimit;
            motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        motor = new TalonFX(motorCanId);

        motorConfiguration.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        motorConfiguration.MotorOutput.Inverted = gearRatio.steerInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.03;
        TalonFXUtils.throwIfError(motor.getConfigurator().apply(motorConfiguration));

        final double appliedMotAngleRot = absoluteEncoder.getAbsoluteAngle().rotations() / steerMotorToMechanismReduction; // rotations du mecanisme
        TalonFXUtils.throwIfError(motor.getPosition().setUpdateFrequency(20, CAN_TIMEOUT_S)); // rotations du moteur
        TalonFXUtils.throwIfError(motor.setPosition(appliedMotAngleRot, CAN_TIMEOUT_S)); // rotations du moteur
        SystemUtils.waitUntil(
            "setPosition for steer encoder " + motorCanId,
            SETTINGS_APPLIED_WAIT_TIMEOUT_MS, 
            () -> MathUtils.areApproxEqual(appliedMotAngleRot, motor.getPosition().getValueAsDouble()) // rotations du moteur
        );
    }

    @Override
    public ContinuousAngle getReferenceAngle() {
        return referenceAngle;
    }

    @Override
    public void setReferenceAngle(ContinuousAngle referenceAngle){
        TalonFXUtils.throwIfError(motor.setControl(positionVoltage.withPosition(referenceAngle.rotations() / steerMotorToMechanismReduction)));
        this.referenceAngle = referenceAngle;
    }

    @Override
    public ContinuousAngle getAngle() {
        return ContinuousAngle.fromRotations(motor.getPosition().getValueAsDouble() * steerMotorToMechanismReduction);
    }

    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return absoluteEncoder.getAbsoluteAngle();
    }

    @Override
    public double getOutput() {
        return motor.getClosedLoopOutput().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // Nothing to do
    }
}
