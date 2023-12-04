package frc.robot.swervelib.ctre;

import frc.robot.swervelib.DriveConfiguration;

public class TalonFXDriveConfiguration implements DriveConfiguration {
    private static final double DEFAULT_NOMINAL_VOLTAGE = 12;
    private static final double DEFAULT_CURRENT_LIMIT = 20;

    public final double nominalVoltage;
    public final double currentLimit;
    public final double feedForwardConstant;
    public final double proportionalConstant;
    public final double integralConstant;
    public final double derivativeConstant;
    public final double velocityConstant;;
    public final double accelerationConstant;
    public final double staticConstant;
    public final double ticksPerMeter;

    public TalonFXDriveConfiguration(
        double nominalVoltage,
        double currentLimit,
        double feedForwardConstant,
        double proportionalConstant,
        double integralConstant,
        double derivativeConstant,
        double velocityConstant,
        double accelerationConstant,
        double staticConstant,
        double ticksPerMeter) {
        this.nominalVoltage = nominalVoltage;
        this.currentLimit = currentLimit;
        this.feedForwardConstant = feedForwardConstant;
        this.proportionalConstant = proportionalConstant;
        this.integralConstant = integralConstant;
        this.derivativeConstant = derivativeConstant;
        this.velocityConstant = velocityConstant;
        this.accelerationConstant = accelerationConstant;
        this.staticConstant = staticConstant;
        this.ticksPerMeter = ticksPerMeter;
    }

    public TalonFXDriveConfiguration() {
        this(
            DEFAULT_NOMINAL_VOLTAGE,
            DEFAULT_CURRENT_LIMIT,
            Double.NaN,
            Double.NaN,
            Double.NaN,
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

    public TalonFXDriveConfiguration withVoltageCompensation(double nominalVoltage) {
        return new TalonFXDriveConfiguration(
                nominalVoltage,
                this.currentLimit,
                this.feedForwardConstant,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant,
                this.velocityConstant,
                this.accelerationConstant,
                this.staticConstant,
                this.ticksPerMeter
        );
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public TalonFXDriveConfiguration withCurrentLimit(double currentLimit) {
        return new TalonFXDriveConfiguration(
                this.nominalVoltage,
                currentLimit,
                this.feedForwardConstant,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant,
                this.velocityConstant,
                this.accelerationConstant,
                this.staticConstant,
                this.ticksPerMeter
        );
    }

    public TalonFXDriveConfiguration withPIDConstants(double feedForwardConstant, double proportional, double integral, double derivative) {
        return new TalonFXDriveConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                feedForwardConstant,
                proportional,
                integral,
                derivative,
                this.velocityConstant,
                this.accelerationConstant,
                this.staticConstant,
                this.ticksPerMeter
        );
    }

    public TalonFXDriveConfiguration withPIDConstants(double proportional, double integral, double derivative) {
        return new TalonFXDriveConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                Double.NaN,
                proportional,
                integral,
                derivative,
                this.velocityConstant,
                this.accelerationConstant,
                this.staticConstant,
                this.ticksPerMeter
        );
    }

    public TalonFXDriveConfiguration withTicksPerMeter(double ticksPerMeter) {
        return new TalonFXDriveConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                this.feedForwardConstant,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant,
                this.velocityConstant,
                this.accelerationConstant,
                this.staticConstant,
                ticksPerMeter
        );
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }

    public TalonFXDriveConfiguration withMotionMagic(double velocityConstant, double accelerationConstant, double staticConstant) {
        return new TalonFXDriveConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                this.feedForwardConstant,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant,
                velocityConstant,
                accelerationConstant,
                staticConstant,
                this.ticksPerMeter
        );
    }

    public boolean hasMotionMagic() {
        return Double.isFinite(velocityConstant) && Double.isFinite(accelerationConstant) && Double.isFinite(staticConstant);
    }
}
