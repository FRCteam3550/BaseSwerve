package swervelib;

/**
 * An angle in the ]-inf, +inf[ range.
 */
public class ContinuousAngle {
    private final double angleDegrees;
    private final double angleRadians;

    private ContinuousAngle(double angleDegrees, double angleRadians) {
        this.angleDegrees = angleDegrees;
        this.angleRadians = angleRadians;
    }

    public static ContinuousAngle fromRadians(double angleRadians) {
        return new ContinuousAngle(Math.toDegrees(angleRadians), angleRadians);
    }

    public static ContinuousAngle fromDegrees(double angleDegrees) {
        return new ContinuousAngle(angleDegrees, Math.toRadians(angleDegrees));
    }
    
    public double radians() {
        return angleRadians;
    }

    public double degrees() {
        return angleDegrees;
    }

    public DiscreetAngle asDiscreet() {
        return DiscreetAngle.fromDegrees(angleDegrees);
    }

    public ContinuousAngle plus(ContinuousAngle other) {
        return new ContinuousAngle(angleDegrees + other.angleDegrees, angleRadians + other.angleRadians);
    }
}
