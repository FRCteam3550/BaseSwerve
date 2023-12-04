package frc.robot.swervelib.rev;

import frc.robot.swervelib.AbsoluteEncoder;
import frc.robot.swervelib.SteerConfiguration;

public class SparkMaxSteerConfiguration implements SteerConfiguration{
    private static final double DEFAULT_NOMINAL_VOLTAGE = 12;
    private static final double DEFAULT_CURRENT_LIMIT = 20;

    public final AbsoluteEncoder absoluteEncoder;
    public final double nominalVoltage;
    public final double currentLimit;
    public final double proportionalConstant;
    public final double integralConstant;
    public final double derivativeConstant;

    public SparkMaxSteerConfiguration(
        AbsoluteEncoder absoluteEncoder,
        double nominalVoltage,
        double currentLimit,
        double proportionalConstant,
        double integralConstant,
        double derivativeConstant) {
        this.absoluteEncoder = absoluteEncoder;
        this.nominalVoltage = nominalVoltage;
        this.currentLimit = currentLimit;
        this.proportionalConstant = proportionalConstant;
        this.integralConstant = integralConstant;
        this.derivativeConstant = derivativeConstant;
    }

    public SparkMaxSteerConfiguration(AbsoluteEncoder absoluteEncoder) {
        this(
                absoluteEncoder,
                DEFAULT_NOMINAL_VOLTAGE,
                DEFAULT_CURRENT_LIMIT,
                Double.NaN,
                Double.NaN,
                Double.NaN
        );
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public SparkMaxSteerConfiguration withVoltageCompensation(double nominalVoltage) {
        return new SparkMaxSteerConfiguration(
                this.absoluteEncoder,
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
                this.absoluteEncoder,
                this.nominalVoltage,
                currentLimit,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant
        );
    }

    public SparkMaxSteerConfiguration withPidConstants(double proportional, double integral, double derivative) {
        return new SparkMaxSteerConfiguration(
                this.absoluteEncoder,
                this.nominalVoltage,
                this.currentLimit,
                proportional,
                integral,
                derivative
        );
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }
}
