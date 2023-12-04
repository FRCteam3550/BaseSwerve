package frc.robot;

import frc.robot.lib.swervelib.DiscreetAngle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int BRAKE_MODE_LIMITSWITCH_ID = 8;
    
    public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 3;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 7;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 9;
    public static final DiscreetAngle FRONT_LEFT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(63.38);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 12;
    public static final DiscreetAngle FRONT_RIGHT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(13.65);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 2;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 8;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 11;
    public static final DiscreetAngle BACK_LEFT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(352.79);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 6;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 4;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 0;
    public static final DiscreetAngle BACK_RIGHT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(212.69);
}
