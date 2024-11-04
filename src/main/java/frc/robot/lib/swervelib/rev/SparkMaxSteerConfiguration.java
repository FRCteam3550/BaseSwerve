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
    public final double proportionalGain;
    public final double integralGain;
    public final double derivativeGain;
    public final boolean useInternalAbsoluteEncoderForFeedback;

    public SparkMaxSteerConfiguration(
        double nominalVoltage,
        double currentLimit,
        double proportionalGain,
        double integralGain,
        double derivativeGain,
        boolean useInternalAbsoluteEncoderForFeedback) {
        this.nominalVoltage = nominalVoltage;
        this.currentLimit = currentLimit;
        this.proportionalGain = proportionalGain;
        this.integralGain = integralGain;
        this.derivativeGain = derivativeGain;
        this.useInternalAbsoluteEncoderForFeedback = useInternalAbsoluteEncoderForFeedback;
    }

    public SparkMaxSteerConfiguration() {
        this(
                DEFAULT_NOMINAL_VOLTAGE,
                DEFAULT_CURRENT_LIMIT,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                false
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
                this.proportionalGain,
                this.integralGain,
                this.derivativeGain,
                this.useInternalAbsoluteEncoderForFeedback
        );
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public SparkMaxSteerConfiguration withCurrentLimit(double currentLimit) {
        return new SparkMaxSteerConfiguration(
                this.nominalVoltage,
                currentLimit,
                this.proportionalGain,
                this.integralGain,
                this.derivativeGain,
                this.useInternalAbsoluteEncoderForFeedback
        );
    }

    public SparkMaxSteerConfiguration withPidGains(double proportional, double integral, double derivative) {
        return new SparkMaxSteerConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                proportional,
                integral,
                derivative,
                this.useInternalAbsoluteEncoderForFeedback
        );
    }

    public SparkMaxSteerConfiguration useInternalAbsoluteEncoderForFeedback() {
        return new SparkMaxSteerConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                this.proportionalGain,
                this.integralGain,
                this.derivativeGain,
                true
        );
    }

    public void ensureHasPidGains() {
        if (!Double.isFinite(proportionalGain) || !Double.isFinite(integralGain) || !Double.isFinite(derivativeGain)) {
            throw new IllegalArgumentException("You must define PID gains for a SparkMaxSteerConfiguration using .withPidGains()");
        }
    }
}
