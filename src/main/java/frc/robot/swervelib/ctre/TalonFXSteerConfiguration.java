package frc.robot.swervelib.ctre;

import frc.robot.swervelib.SteerConfiguration;

public class TalonFXSteerConfiguration implements SteerConfiguration {
    private static final double DEFAULT_NOMINAL_VOLTAGE = 12;
    private static final double DEFAULT_CURRENT_LIMIT = 20;

    public final double nominalVoltage;
    public final double currentLimit;
    public final double proportionalConstant;
    public final double integralConstant;
    public final double derivativeConstant;
    public final double velocityConstant;
    public final double accelerationConstant;
    public final double staticConstant;

    public TalonFXSteerConfiguration(
        double nominalVoltage,
        double currentLimit,
        double proportionalConstant,
        double integralConstant,
        double derivativeConstant,
        double velocityConstant,
        double accelerationConstant,
        double staticConstant) {
        this.nominalVoltage = nominalVoltage;
        this.currentLimit = currentLimit;
        this.proportionalConstant = proportionalConstant;
        this.integralConstant = integralConstant;
        this.derivativeConstant = derivativeConstant;
        this.velocityConstant = velocityConstant;
        this.accelerationConstant = accelerationConstant;
        this.staticConstant = staticConstant;
    }

    public TalonFXSteerConfiguration() {
        this(
            DEFAULT_NOMINAL_VOLTAGE,
            DEFAULT_CURRENT_LIMIT,
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

    public TalonFXSteerConfiguration withVoltageCompensation(double nominalVoltage) {
        return new TalonFXSteerConfiguration(
                nominalVoltage,
                this.currentLimit,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant,
                this.velocityConstant,
                this.accelerationConstant,
                this.staticConstant
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
                this.derivativeConstant,
                this.velocityConstant,
                this.accelerationConstant,
                this.staticConstant
        );
    }

    public TalonFXSteerConfiguration withPidConstants(double proportional, double integral, double derivative) {
        return new TalonFXSteerConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                proportional,
                integral,
                derivative,
                this.velocityConstant,
                this.accelerationConstant,
                this.staticConstant
        );
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }

    public TalonFXSteerConfiguration withMotionMagic(double velocityConstant, double accelerationConstant, double staticConstant) {
        return new TalonFXSteerConfiguration(
                this.nominalVoltage,
                this.currentLimit,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant,
                velocityConstant,
                accelerationConstant,
                staticConstant
        );
    }

    public boolean hasMotionMagic() {
        return Double.isFinite(velocityConstant) && Double.isFinite(accelerationConstant) && Double.isFinite(staticConstant);
    }
}
