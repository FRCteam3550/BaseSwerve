package frc.robot.lib;

public class MathUtils {
    public static final double DEFAULT_EPSILON = 1e-4;

    public static final boolean areApproxEqual(double x, double y) {
        return areApproxEqual(x, y, DEFAULT_EPSILON);
    }
    
    public static final boolean areApproxEqual(double x, double y, double epsilon) {
        return Math.abs(x - y) < epsilon;
    }
}
