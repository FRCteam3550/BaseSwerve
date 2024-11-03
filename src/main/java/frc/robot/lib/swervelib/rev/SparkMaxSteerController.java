package frc.robot.lib.swervelib.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkLowLevel;

import frc.robot.lib.MathUtils;
import frc.robot.lib.SparkMaxUtils;
import frc.robot.lib.SystemUtils;
import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.ContinuousAngle;
import frc.robot.lib.swervelib.DiscreetAngle;
import frc.robot.lib.swervelib.GearRatio;
import frc.robot.lib.swervelib.SteerController;

public final class SparkMaxSteerController implements SteerController {
    private static final long SETTINGS_APPLIED_WAIT_TIMEOUT_MS = 500;
    private final CANSparkMax motor;
    private final SparkPIDController pidController;
    private final RelativeEncoder motorEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    private ContinuousAngle referenceAngle = ContinuousAngle.fromDegrees(0);

    public SparkMaxSteerController(int motorCanId, SparkMaxSteerConfiguration steerConfiguration, GearRatio gearRatio, AbsoluteEncoder absoluteEncoder) {
        steerConfiguration.ensureHasPidConstants();
        this.absoluteEncoder = absoluteEncoder;

        motor = SparkMaxUtils.getController(motorCanId);
        SparkMaxUtils.throwIfError(motor.restoreFactoryDefaults());
        SparkMaxUtils.throwIfError(motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100));
        SparkMaxUtils.throwIfError(motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20));
        SparkMaxUtils.throwIfError(motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20));
        SparkMaxUtils.throwIfError(motor.setIdleMode(CANSparkMax.IdleMode.kBrake));
        motor.setInverted(!gearRatio.steerInverted);
        SparkMaxUtils.throwIfError(motor.setSmartCurrentLimit(38));
        if (steerConfiguration.hasVoltageCompensation()) {
            SparkMaxUtils.throwIfError(motor.enableVoltageCompensation(steerConfiguration.nominalVoltage));
        }
        if (steerConfiguration.hasCurrentLimit()) {
            SparkMaxUtils.throwIfError(motor.setSmartCurrentLimit((int) Math.round(steerConfiguration.currentLimit)));
        }

        motorEncoder = motor.getEncoder();
        double positionToDegreesRatio = 360 * gearRatio.steerMotorToMechanismReduction;
        SparkMaxUtils.throwIfError(motorEncoder.setPositionConversionFactor(positionToDegreesRatio));        
        SystemUtils.waitUntil(
            SETTINGS_APPLIED_WAIT_TIMEOUT_MS,
            () -> MathUtils.areApproxEqual(positionToDegreesRatio, motorEncoder.getPositionConversionFactor())
        );
        SparkMaxUtils.throwIfError(motorEncoder.setPosition(absoluteEncoder.getAbsoluteAngle().degrees()));
        
        pidController = motor.getPIDController();
        SparkMaxUtils.throwIfError(pidController.setP(steerConfiguration.proportionalConstant));
        SparkMaxUtils.throwIfError(pidController.setI(steerConfiguration.integralConstant));
        SparkMaxUtils.throwIfError(pidController.setD(steerConfiguration.derivativeConstant));
        SparkMaxUtils.throwIfError(motor.burnFlash());
    }

    @Override
    public ContinuousAngle getReferenceAngle() {
        return referenceAngle;
    }

    @Override
    public void setReferenceAngle(ContinuousAngle referenceAngle) {
        this.referenceAngle = referenceAngle;
        pidController.setReference(referenceAngle.degrees(), CANSparkBase.ControlType.kPosition);
    }

    @Override
    public ContinuousAngle getAngle() {
        return ContinuousAngle.fromDegrees(motorEncoder.getPosition());
    }

    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return absoluteEncoder.getAbsoluteAngle();
    }

    @Override
    public double getOutput() {
        return motor.getAppliedOutput();
    }

    private static final double ENCODER_RESOLUTION_DEG = 360.0 * (1.0 / 42.0);
    private static final double REFERENCE_TOLERANCE_DEG = ENCODER_RESOLUTION_DEG;
    private boolean isAtReference() {
        var actualDeg = motor.getEncoder().getPosition();
        var desiredDeg = referenceAngle.degrees();
        return Math.abs(desiredDeg - actualDeg) <= REFERENCE_TOLERANCE_DEG;
    }

    @Override
    public void periodic() {
        if (isAtReference()) {
            motor.stopMotor();
        }
    }
}
