package frc.robot.lib.swervelib.rev;

import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.GearRatio;
import frc.robot.lib.swervelib.SteerConfiguration;
import frc.robot.lib.swervelib.SteerController;

public class SparkMaxSteerConfiguration implements SteerConfiguration{
    private static final double DEFAULT_NOMINAL_VOLTAGE = 12;
    private static final double DEFAULT_CURRENT_LIMIT = 20;

    public final double nominalVoltage;
    public final double currentLimit;
    public final double proportionalConstant;
    public final double integralConstant;
    public final double derivativeConstant;

    public SparkMaxSteerConfiguration(
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

    public SparkMaxSteerConfiguration() {
        this(
                DEFAULT_NOMINAL_VOLTAGE,
                DEFAULT_CURRENT_LIMIT,
                Double.NaN,
                Double.NaN,
                Double.NaN
        );
    }

    public SteerController createSteerController(int motorCanId, GearRatio gearRatio, AbsoluteEncoder absoluteEncoder) {
        return new SparkMaxSteerController(motorCanId, this, gearRatio, absoluteEncoder);
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public SparkMaxSteerConfiguration withVoltageCompensation(double nominalVoltage) {
        return new SparkMaxSteerConfiguration(
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

    public SparkMaxSteerConfiguration withCurrentLimit(double currentLimit) {
        return new SparkMaxSteerConfiguration(
                this.nominalVoltage,
                currentLimit,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant
        );
    }

    public SparkMaxSteerConfiguration withPidConstants(double proportional, double integral, double derivative) {
        return new SparkMaxSteerConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                proportional,
                integral,
                derivative
        );
    }

    public void ensureHasPidConstants() {
        if (!Double.isFinite(proportionalConstant) || !Double.isFinite(integralConstant) || !Double.isFinite(derivativeConstant)) {
            throw new IllegalArgumentException("You must define PID parameter for a SparkMaxSteerConfiguration using .withPidConstants()");
        }
    }
}
