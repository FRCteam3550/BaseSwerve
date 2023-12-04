package frc.robot.lib.swervelib;

public class SwerveModule {
    private final DriveController driveController;
    private final SteerController steerController;
    private static final double TWO_PI = 2 * Math.PI;
    private static final double HALF_PI = Math.PI / 2.0;

    public SwerveModule(DriveController driveController,
                               SteerController steerController) {
        this.driveController = driveController;
        this.steerController = steerController;
    }

    public DiscreetAngle getSteerAbsoluteAngle() {
        return steerController.getAbsoluteAngle();
    }

    public ContinuousAngle getSteerAngle() {
        return steerController.getAngle();
    }

    public double getDrivePositionM() {
        return driveController.getPositionM();
    }

    public double getDrivePositionTicks() {
        return driveController.getPositionTicks();
    }

    public Double getDriveSpeedMS() {
        return driveController.getSpeedMS();
    }

    public double getReferenceSpeedMS() {
        return driveController.getReferenceSpeedMS();
    }

    public ContinuousAngle getSteerReferenceAngle() {
        return steerController.getReferenceAngle();
    }

    public void setOpenLoopSpeed(double drivePct, DiscreetAngle steerAngleRadians) {
        var steerSetPoint = getSteerAngleAndDriveSign(steerAngleRadians, getSteerAngle());
        steerController.setReferenceAngle(steerSetPoint.targetAngle);
        driveController.setOpenLoopSpeed(drivePct * steerSetPoint.driveSign);
    }

    public void setClosedLoopSpeed(double driveMS, DiscreetAngle steerAngleRadians) {
        var steerSetPoint = getSteerAngleAndDriveSign(steerAngleRadians, getSteerAngle());
        steerController.setReferenceAngle(steerSetPoint.targetAngle);
        driveController.setClosedLoopSpeed(driveMS * steerSetPoint.driveSign);
    }    

    static SteerSetPoint getSteerAngleAndDriveSign(DiscreetAngle targetAngle, ContinuousAngle currentAngle) {
        var driveSign = 1.0;

        double difference = targetAngle.radians() - currentAngle.asDiscreet().radians();
        // Change the target angle so the difference is in the range [-pi, pi) instead of [-2pi, 2pi)
        if (difference >= Math.PI) {
            difference -= TWO_PI;
        } else if (difference < -Math.PI) {
            difference += TWO_PI;
        }

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > HALF_PI) {
            difference -= Math.PI;
            driveSign = -1.0;
        } else if (difference < -HALF_PI) {
            difference += Math.PI;
            driveSign = -1.0;
        }

        return new SteerSetPoint(
            driveSign, 
            currentAngle.plus(ContinuousAngle.fromRadians(difference))
        );
    }
}
