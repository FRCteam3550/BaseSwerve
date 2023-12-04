package frc.robot.lib.swervelib.rev;

import frc.robot.lib.swervelib.DriveConfiguration;

public class SparkMaxDriveConfiguration implements DriveConfiguration {
    private static final double DEFAULT_NOMINAL_VOLTAGE = 12;
    private static final double DEFAULT_DRIVE_CURRENT_LIMIT = 80;

    public final double nominalVoltage;
    public final double currentLimit;
    public final double feedForwardConstant;
    public final double proportionalConstant;
    public final double integralConstant;
    public final double derivativeConstant;
    public final double rotationsPerMeter;

    public SparkMaxDriveConfiguration(
        double nominalVoltage,
        double currentLimit,
        double feedForwardConstant,
        double proportionalConstant,
        double integralConstant,
        double derivativeConstant,
        double rotationsPerMeter) {
        this.nominalVoltage = nominalVoltage;
        this.currentLimit = currentLimit;
        this.feedForwardConstant = feedForwardConstant;
        this.proportionalConstant = proportionalConstant;
        this.integralConstant = integralConstant;
        this.derivativeConstant = derivativeConstant;
        this.rotationsPerMeter = rotationsPerMeter;
    }

    public SparkMaxDriveConfiguration() {
        this(
            DEFAULT_NOMINAL_VOLTAGE,
            DEFAULT_DRIVE_CURRENT_LIMIT,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN
        );
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public SparkMaxDriveConfiguration withVoltageCompensation(double nominalVoltage) {
        return new SparkMaxDriveConfiguration(
            nominalVoltage,
            this.currentLimit,
            this.feedForwardConstant,
            this.proportionalConstant,
            this.integralConstant,
            this.derivativeConstant,
            this.rotationsPerMeter
        );
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public SparkMaxDriveConfiguration withCurrentLimit(double currentLimit) {
        return new SparkMaxDriveConfiguration(
            this.nominalVoltage,
            currentLimit,
            this.feedForwardConstant,
            this.proportionalConstant,
            this.integralConstant,
            this.derivativeConstant,
            this.rotationsPerMeter
        );
    }

    public SparkMaxDriveConfiguration withPIDConstants(double feedForwardConstant, double proportional, double integral, double derivative) {
        return new SparkMaxDriveConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                feedForwardConstant,
                proportional,
                integral,
                derivative,
                this.rotationsPerMeter
        );
    }
    
    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }

    public SparkMaxDriveConfiguration withTicksPerMeter(double ticksPerMeter) {
        return new SparkMaxDriveConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                this.feedForwardConstant,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant,
                ticksPerMeter
        );
    }

}
