package frc.robot.lib.swervelib.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
    private final SparkMax motor;
    private final SparkClosedLoopController pidController;
        private final SparkMaxConfig config = new SparkMaxConfig();
    private final RelativeEncoder motorEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    private ContinuousAngle referenceAngle = ContinuousAngle.fromDegrees(0);

    public SparkMaxSteerController(int motorCanId, SparkMaxSteerConfiguration steerConfiguration, GearRatio gearRatio, AbsoluteEncoder absoluteEncoder) {
        steerConfiguration.ensureHasPidConstants();
        this.absoluteEncoder = absoluteEncoder;

        motor = SparkMaxUtils.getController(motorCanId); // Already reset to factory defaults
        SparkMaxUtils.throwIfREVLibError(motor.clearFaults());
        config.signals.primaryEncoderPositionPeriodMs(20);
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        config.smartCurrentLimit(38);
        if (steerConfiguration.hasVoltageCompensation()) {
           config.voltageCompensation(steerConfiguration.nominalVoltage);
        }
        if (steerConfiguration.hasCurrentLimit()) {
            config.smartCurrentLimit((int)steerConfiguration.currentLimit);
        }

        motorEncoder = motor.getEncoder();
        double positionToDegreesRatio = 360 * gearRatio.steerMotorToMechanismReduction;
        config.absoluteEncoder.positionConversionFactor(positionToDegreesRatio);    
        SystemUtils.waitUntil(
            "setPositionConversionFactor for steer encoder " + motorCanId,
            SETTINGS_APPLIED_WAIT_TIMEOUT_MS,
            () -> MathUtils.areApproxEqual(positionToDegreesRatio, motor.configAccessor.absoluteEncoder.getPositionConversionFactor())
        );
        SparkMaxUtils.throwIfREVLibError(motorEncoder.setPosition(absoluteEncoder.getAbsoluteAngle().degrees()));
        
        pidController = motor.getClosedLoopController();
        config.closedLoop.pid(
            steerConfiguration.proportionalConstant, 
            steerConfiguration.integralConstant, 
            steerConfiguration.derivativeConstant
        );
        SparkMaxUtils.throwIfREVLibError(motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public ContinuousAngle getReferenceAngle() {
        return referenceAngle;
    }

    @Override
    public void setReferenceAngle(ContinuousAngle referenceAngle) {
        this.referenceAngle = referenceAngle;
        if (isAtReference()) {
            motor.stopMotor();
        }
        else {
            pidController.setReference(referenceAngle.degrees(), SparkBase.ControlType.kPosition);
        }
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
        return MathUtils.areApproxEqual(
            motor.getEncoder().getPosition(),
            referenceAngle.degrees(),
            REFERENCE_TOLERANCE_DEG
        );
    }

    @Override
    public void periodic() {
    }
}
