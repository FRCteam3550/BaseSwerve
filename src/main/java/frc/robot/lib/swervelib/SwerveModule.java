package frc.robot.lib.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveSpeedMS(),
            Rotation2d.fromRadians(getSteerAngle().radians())
        );
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

    public double getDrivePositionNativeUnits() {
        return driveController.getPositionNativeUnits();
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

    public double getSteerOuptut() {
        return steerController.getOutput();
    }

    public double getDriveOuptut() {
        return driveController.getOutput();
    }

    public void setOpenLoopSpeed(double drivePct, DiscreetAngle steerAngle) {
        var steerSetPoint = getSteerAngleAndDriveSign(steerAngle, getSteerAngle());
        steerController.setReferenceAngle(steerSetPoint.targetAngle);
        driveController.setOpenLoopSpeed(drivePct * steerSetPoint.driveSign);
    }

    public void setClosedLoopSpeed(double driveMS, DiscreetAngle steerAngle) {
        var steerSetPoint = getSteerAngleAndDriveSign(steerAngle, getSteerAngle());
        steerController.setReferenceAngle(steerSetPoint.targetAngle);
        driveController.setClosedLoopSpeed(driveMS * steerSetPoint.driveSign);
    }    

    static SteerSetPoint getSteerAngleAndDriveSign(DiscreetAngle targetAngle, ContinuousAngle currentAngle) {
        var driveSign = 1.0;
        double differenceRadians = targetAngle.radians() - currentAngle.asDiscreet().radians();

        // Change the target angle so the difference is in the range [-180, 180) deg instead of [-360, 360) deg
        if (differenceRadians >= Math.PI) {
            differenceRadians -= TWO_PI;
        } else if (differenceRadians < -Math.PI) {
            differenceRadians += TWO_PI;
        }

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (differenceRadians > HALF_PI) {
            differenceRadians -= Math.PI;
            driveSign = -1.0;
        } else if (differenceRadians < -HALF_PI) {
            differenceRadians += Math.PI;
            driveSign = -1.0;
        }

        return new SteerSetPoint(
            driveSign, 
            currentAngle.plus(ContinuousAngle.fromRadians(differenceRadians))
        );
    }

    public void periodic() {
        steerController.periodic();
    }
}
