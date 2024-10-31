package frc.robot.lib.swervelib.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

import com.revrobotics.CANSparkLowLevel;

import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.ContinuousAngle;
import frc.robot.lib.swervelib.DiscreetAngle;
import frc.robot.lib.swervelib.GearRatio;
import frc.robot.lib.swervelib.SteerController;

public final class SparkMaxSteerController implements SteerController {
    private final CANSparkMax motor;
    private final SparkPIDController controller;
    private final RelativeEncoder motorEncoder;
    private final AbsoluteEncoder absoluteEncoder;
    private final PIDController wpiPid;
    private double wpiOutput = 0;

    private static final long WAIT_TIME_MS = 10;
    private static final double EPSILON = 1e-4;

    private ContinuousAngle referenceAngle = ContinuousAngle.fromDegrees(0);

    public boolean hasShift = false; 
    public double shift = 0.0;

    public SparkMaxSteerController(int motorCanId, SparkMaxSteerConfiguration steerConfiguration, GearRatio gearRatio, AbsoluteEncoder absoluteEncoder) {
        this.absoluteEncoder = absoluteEncoder;

        motor = new CANSparkMax(motorCanId, CANSparkLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setInverted(!gearRatio.steerInverted);
        motor.setSmartCurrentLimit(38);
        if (steerConfiguration.hasVoltageCompensation()) {
            motor.enableVoltageCompensation(steerConfiguration.nominalVoltage);
        }
        if (steerConfiguration.hasCurrentLimit()) {
            motor.setSmartCurrentLimit((int) Math.round(steerConfiguration.currentLimit));
        }

        motorEncoder = motor.getEncoder();
        double positionRatio = 360 * gearRatio.steerMotorToMechanismReduction;
        motorEncoder.setPositionConversionFactor(positionRatio);
        motorEncoder.setVelocityConversionFactor(positionRatio / 60.0);
        
        waitFor(20);
        final double appliedAngle = motorCanId == 4 ? getInitAngleDeg() : absoluteEncoder.getAbsoluteAngle().degrees();
        throwIfError(motorEncoder.setPosition(appliedAngle));
        
        controller = motor.getPIDController();
        if (steerConfiguration.hasPidConstants()) {
            // controller.setP(steerConfiguration.proportionalConstant);
            // controller.setI(steerConfiguration.integralConstant);
            // controller.setD(steerConfiguration.derivativeConstant);
            wpiPid = new PIDController(steerConfiguration.proportionalConstant, steerConfiguration.integralConstant, steerConfiguration.derivativeConstant);
            wpiPid.setTolerance(0.5);
        }
        else {
            wpiPid = new PIDController(0,0,0);
        }
        throwIfError(motor.burnFlash());
    }

    public void resetEncoder() {
        var initAngle = absoluteEncoder.getAbsoluteAngle().degrees();
        throwIfError(motorEncoder.setPosition(initAngle));
    }

    @Override
    public ContinuousAngle getReferenceAngle() {
        return referenceAngle;
    }

    @Override
    public void setReferenceAngle(ContinuousAngle referenceAngle) {
        this.referenceAngle = referenceAngle;
        //controller.setReference(referenceAngle.degrees(), CANSparkBase.ControlType.kPosition);
    }

    @Override
    public ContinuousAngle getAngle() {
        return ContinuousAngle.fromDegrees(motorEncoder.getPosition());
    }

    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return absoluteEncoder.getAbsoluteAngle();
    }

    private static final void throwIfError(REVLibError error) {
        if (error != REVLibError.kOk) {
            throw new RuntimeException(String.format("Error: %s", error.name()));
        }
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
            DriverStation.reportWarning("Waited too long in SparkMaxSteerController!", false);
        }

        return totalWaitTimeMs;
    }

    private static final boolean areApproxEqual(double x, double y) {
        return Math.abs(x - y) < EPSILON;
    }
    
    private double getInitAngleDeg() {
        var initAngle = absoluteEncoder.getAbsoluteAngle().degrees();

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
        double currentAngleDeg = motorEncoder.getPosition() + 180;
        throwIfError(motorEncoder.setPosition(currentAngleDeg));
        waitUntil(500, () -> areApproxEqual(currentAngleDeg, motorEncoder.getPosition()));
    }

    @Override
    public double getOutput() {
        //return motor.getAppliedOutput();
        return wpiOutput;
    }

    @Override
    public void periodic() {
        wpiPid.setSetpoint(referenceAngle.degrees());
        if (wpiPid.atSetpoint()) {
            motor.stopMotor();
            wpiPid.reset();
        }
        else {
            wpiOutput = wpiPid.calculate(motorEncoder.getPosition());
            motor.set(wpiOutput);
        }
    }
}
