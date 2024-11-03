package frc.robot.lib.swervelib;

public final class RevGearRatios {
    private static double SWERVE_MAX_3IN_WHEEL_CIRCUMFERENCE_M = 0.0762 * Math.PI; // 0.2394m

    private static double SWERVE_MAX_3IN_STEERING_REDUCTION = (1.0 / 2.89) * (1.0 / 3.61) * (14.0 / 62.0); // 0.02164
    //private static double SWERVE_MAX_3IN_STEERING_REDUCTION = 0.02141;  // Measured

    public static final GearRatio SWERVE_MAX_3IN_LOW = new GearRatio(
        SWERVE_MAX_3IN_WHEEL_CIRCUMFERENCE_M,
        (12.0 / 22.0) * (15.0 / 45.0),
        false,
        SWERVE_MAX_3IN_STEERING_REDUCTION,
        false
    );

    public static final GearRatio SWERVE_MAX_3IN_MID = new GearRatio(
        SWERVE_MAX_3IN_WHEEL_CIRCUMFERENCE_M,
        (13.0 / 22.0) * (15.0 / 45.0),
        false,
        SWERVE_MAX_3IN_STEERING_REDUCTION,
        false
    );

    public static final GearRatio SWERVE_MAX_3IN_HIGH = new GearRatio(
        SWERVE_MAX_3IN_WHEEL_CIRCUMFERENCE_M,
        (14.0 / 22.0) * (15.0 / 45.0),
        false,
        SWERVE_MAX_3IN_STEERING_REDUCTION,
        false
    );

    private RevGearRatios() {
    }
}
