package frc.robot.lib.swervelib;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import static org.junit.jupiter.api.Assertions.*;


public class SwerveModuleTest {
    static final double EPSILON = 0.0000001;
    private static final double FRONT_SIDE_M = .47;
    private static final double RIGHT_SIDE_M = .47;
    private static final double BACK_SIDE_M = .47;
    private static final double LEFT_SIDE_M = .47;

    private static final double FRONT_RIGHT_MODULE_X_M = FRONT_SIDE_M / 2;
    private static final double FRONT_RIGHT_MODULE_Y_M = -RIGHT_SIDE_M / 2;

    private static final double FRONT_LEFT_MODULE_X_M = FRONT_SIDE_M / 2;
    private static final double FRONT_LEFT_MODULE_Y_M = LEFT_SIDE_M / 2;

    private static final double BACK_RIGHT_MODULE_X_M = -BACK_SIDE_M / 2;
    private static final double BACK_RIGHT_MODULE_Y_M = -RIGHT_SIDE_M / 2;

    private static final double BACK_LEFT_MODULE_X_M = -BACK_SIDE_M / 2;
    private static final double BACK_LEFT_MODULE_Y_M = LEFT_SIDE_M / 2;
    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            // Front left
            new Translation2d(FRONT_LEFT_MODULE_X_M, FRONT_LEFT_MODULE_Y_M),
            // Front right
            new Translation2d(FRONT_RIGHT_MODULE_X_M, FRONT_RIGHT_MODULE_Y_M),
            // Back left
            new Translation2d(BACK_LEFT_MODULE_X_M, BACK_LEFT_MODULE_Y_M),
            // Back right
            new Translation2d(BACK_RIGHT_MODULE_X_M, BACK_RIGHT_MODULE_Y_M));

    static void assertSetpointEquals(double expectedAngleDegrees, double expectedDriveSign, SteerSetPoint actual) {
        assertEquals(expectedAngleDegrees, actual.targetAngle.degrees(), EPSILON);
        assertEquals(expectedDriveSign, actual.driveSign);
    }

    static  SteerSetPoint getSteerAngleAndDriveSign(double targetAngleDegrees, double currentAngleDegrees) {
        return SwerveModule.getSteerAngleAndDriveSign(
            DiscreetAngle.fromDegrees(targetAngleDegrees),
            ContinuousAngle.fromDegrees(currentAngleDegrees)
        );
    }

    @Test
    void testChassisSpeedRot() {
        var chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            0.5 * 10,
            Rotation2d.fromDegrees(0)
        );
        System.out.println(String.format("%s %s %s", chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond, chassisSpeed.omegaRadiansPerSecond));
        var states = KINEMATICS.toSwerveModuleStates(chassisSpeed);
        System.out.println(String.format("avg %s avd %s arg %s ard %s", states[0].angle.getDegrees(), states[1].angle.getDegrees(), states[2].angle.getDegrees(), states[3].angle.getDegrees()));
    }

    @Test
    void testChassisSpeedX() {
        var chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
            1,
            0,
            0,
            Rotation2d.fromDegrees(0)
        );
        System.out.println(String.format("%s %s %s", chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond, chassisSpeed.omegaRadiansPerSecond));
        var states = KINEMATICS.toSwerveModuleStates(chassisSpeed);
        System.out.println(String.format("avg %s avd %s arg %s ard %s", states[0].angle.getDegrees(), states[1].angle.getDegrees(), states[2].angle.getDegrees(), states[3].angle.getDegrees()));
    }

    @Test
    void testChassisSpeedY() {
        var chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            1,
            0,
            Rotation2d.fromDegrees(0)
        );
        System.out.println(String.format("%s %s %s", chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond, chassisSpeed.omegaRadiansPerSecond));
        var states = KINEMATICS.toSwerveModuleStates(chassisSpeed);
        System.out.println(String.format("avg %s avd %s arg %s ard %s", states[0].angle.getDegrees(), states[1].angle.getDegrees(), states[2].angle.getDegrees(), states[3].angle.getDegrees()));
    }

    @Test
    void when0CurrentAngle() {
        var currentDegrees = 0;

        assertSetpointEquals(0, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(90, 1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(0, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(-90, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(-10, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    }    

    @Test
    void when360CurrentAngle() {
        var currentDegrees = 360;

        assertSetpointEquals(360, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(450, 1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(360, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(270, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(350, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    }    

    @Test
    void whenSmallCurrentAngle() {
        var currentDegrees = 10;

        assertSetpointEquals(0, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(90, 1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(0, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(90, -1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(-10, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    }    

    @Test
    void whenLargeCurrentAngle() {
        var currentDegrees = 350;

        assertSetpointEquals(360, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(270, -1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(360, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(270, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(350, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    } 

    @Test
    void whenCurrentAngleMoreThan360() {
        var currentDegrees = 710;

        assertSetpointEquals(720, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(630, -1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(720, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(630, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(710, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    } 

    @Test
    void whenNegativeAngle() {
        var currentDegrees = -360;

        assertSetpointEquals(-360, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(-270, 1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(-360, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(-450, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(-370, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    } 

    @Test
    void whenNegativeAngleTest2() {
        var currentDegrees = -370;

        assertSetpointEquals(-360, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(-450, -1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(-360, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(-450, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(-370, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    } 
}
