package frc.robot.lib.swervelib.ctre;

import java.util.function.BooleanSupplier;

import javax.swing.text.Position;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.TalonFXUtils;
import frc.robot.lib.swervelib.*;

public final class TalonFXSteerController implements SteerController {
    private static final double CAN_TIMEOUT_S = 0.250;
    private final TalonFX motor;
    private final double steerMotorToMechanismReduction;
    private final AbsoluteEncoder absoluteEncoder;

    private static final double EPSILON = 1e-4;
    private static final long WAIT_TIME_MS = 10;

    public boolean hasShift = false; 
    public double shift = 0.0;

    private ContinuousAngle referenceAngle = ContinuousAngle.fromDegrees(0.0);

    private final PositionVoltage positionVoltage = new PositionVoltage(0, 0, false, 0, 0, false, false, false);

    public TalonFXSteerController(int motorCanId, TalonFXSteerConfiguration steerConfiguration, GearRatio gearRatio, AbsoluteEncoder absoluteEncoder) {
        steerMotorToMechanismReduction = gearRatio.steerMotorToMechanismReduction;
        this.absoluteEncoder = absoluteEncoder;
        // motorControlMode = steerConfiguration.hasMotionMagic() ? ControlModeValue.MotionMagicDutyCycle : ControlModeValue.PositionDutyCycle;

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        if (steerConfiguration.hasPidConstants()) {
            motorConfiguration.Slot0.kP = steerConfiguration.proportionalConstant;
            motorConfiguration.Slot0.kI = steerConfiguration.integralConstant;
            motorConfiguration.Slot0.kD = steerConfiguration.derivativeConstant;
        }
        if (steerConfiguration.hasCurrentLimit()) {
            motorConfiguration.CurrentLimits.SupplyCurrentLimit = steerConfiguration.currentLimit;
            motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        motor = new TalonFX(motorCanId);

        motorConfiguration.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        motorConfiguration.MotorOutput.Inverted = gearRatio.steerInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.03;
        final double appliedMotAngleRot = (motorCanId == 4 ? getInitAngleRot() : absoluteEncoder.getAbsoluteAngle().rotations()) / steerMotorToMechanismReduction; //rotations du mecanism
        // motorConfiguration.Feedback.FeedbackRotorOffset = appliedMotAngleRot;
        TalonFXUtils.throwIfError(motor.getConfigurator().apply(motorConfiguration));

        TalonFXUtils.throwIfError(motor.getPosition().setUpdateFrequency(20, CAN_TIMEOUT_S)); //rotations du moteur
        TalonFXUtils.throwIfError(motor.setPosition(appliedMotAngleRot, CAN_TIMEOUT_S)); //rotations du moteur
        waitUntil(500, () -> areApproxEqual(appliedMotAngleRot, motor.getPosition().getValueAsDouble())); //rotations du moteur
    }

    private static final void waitFor(long waitTimeMillis) {
        try {
            Thread.sleep(waitTimeMillis);
        }
        catch(InterruptedException ie) {}
    }

    private static final long waitUntil(long maxWait, BooleanSupplier conditionToBeTrue) {
        long totalWaitTimeMs = 0;

        while (!conditionToBeTrue.getAsBoolean() && totalWaitTimeMs < maxWait) {
            try {
                Thread.sleep(WAIT_TIME_MS);
                totalWaitTimeMs += WAIT_TIME_MS;
            }
            catch(InterruptedException ie) {}
        }

        if (!conditionToBeTrue.getAsBoolean()) {
            DriverStation.reportWarning("Waited too long in TalonFXSteerController!", false);
        }

        return totalWaitTimeMs;
    }

    private static final boolean areApproxEqual(double x, double y) {
        return Math.abs(x - y) < EPSILON;
    }
    
    private double getInitAngleRot() {
        var initAngle = absoluteEncoder.getAbsoluteAngle().rotations();

        hasShift = !(initAngle < 3 || initAngle > 357 || (initAngle > 177 && initAngle < 183));

        if (!hasShift) {
            shift = 0;
            return initAngle;
        } else if (initAngle < 90) {
            shift = initAngle;
            return 0;
        } else if (initAngle < 270) {
            shift = 180 - initAngle;
            return 180;
        } else {
            shift = 360 - initAngle;
            return 0;
        }
    } 

    public void invertWheel() {
        motor.stopMotor();
        double currentAngleDeg = motor.getPosition().getValueAsDouble() + 180;
        TalonFXUtils.throwIfError(motor.setPosition(currentAngleDeg));
        waitUntil(500, () -> areApproxEqual(currentAngleDeg, motor.getPosition().getValueAsDouble()));
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

    }

}
