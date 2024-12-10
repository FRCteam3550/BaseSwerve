package frc.robot.lib.swervelib.ctre;

import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.GearRatio;
import frc.robot.lib.swervelib.SteerConfiguration;
import frc.robot.lib.swervelib.SteerController;

public class TalonFXSteerConfiguration implements SteerConfiguration {
    private static final double DEFAULT_NOMINAL_VOLTAGE = 12;
    private static final double DEFAULT_CURRENT_LIMIT = 20;

    public final double nominalVoltage;
    public final double currentLimit;
    public final double proportionalConstant;
    public final double integralConstant;
    public final double derivativeConstant;

    public TalonFXSteerConfiguration(
        double nominalVoltage,
        double currentLimit,
        double proportionalConstant,
        double integralConstant,
        double derivativeConstant) {
        this.nominalVoltage = nominalVoltage;
        this.currentLimit = currentLimit;
        this.proportionalConstant = proportionalConstant;
        this.integralConstant = integralConstant;
        this.derivativeConstant = derivativeConstant;
    }

    public TalonFXSteerConfiguration() {
        this(
            DEFAULT_NOMINAL_VOLTAGE,
            DEFAULT_CURRENT_LIMIT,
            Double.NaN,
            Double.NaN,
            Double.NaN
        );
    }

    public SteerController createSteerController(int motorCanId, GearRatio gearRatio, AbsoluteEncoder absoluteEncoder) {
        return new TalonFXSteerController(motorCanId, this, gearRatio, absoluteEncoder);
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public TalonFXSteerConfiguration withVoltageCompensation(double nominalVoltage) {
        return new TalonFXSteerConfiguration(
                nominalVoltage,
                this.currentLimit,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant
        );
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public TalonFXSteerConfiguration withCurrentLimit(double currentLimit) {
        return new TalonFXSteerConfiguration(
                this.nominalVoltage,
                currentLimit,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant
        );
    }

    public TalonFXSteerConfiguration withPidConstants(double proportional, double integral, double derivative) {
        return new TalonFXSteerConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                proportional,
                integral,
                derivative
        );
    }

    public void ensureHasPidConstants() {
        if (!Double.isFinite(proportionalConstant) || !Double.isFinite(integralConstant) || !Double.isFinite(derivativeConstant)) {
            throw new IllegalArgumentException("You must define PID parameter for a TalonFXSteerConfiguration using .withPidConstants()");
        }
    }
}
