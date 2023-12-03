package swervelib;

/** Add your docs here. */
public class SteerSetPoint {
    public final double driveSign;
    public final ContinuousAngle targetAngle;

    public SteerSetPoint(double driveSign, ContinuousAngle targetAngle) {
        this.driveSign = driveSign;
        this.targetAngle = targetAngle;
    }
}
