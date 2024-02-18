package frc.robot.lib.swervelib;

public interface DriveController {
    /**
     * Sets the motor's output.
     * @param pct a number between -1 and 1.
     */
    void setOpenLoopSpeed(double pct);

    void setClosedLoopSpeed(double speedMS);

    double getSpeedMS();

    double getPositionM();

    double getPositionNativeUnits();

    double getOpenLoopReferenceSpeedPct();

    double getClosedLoopReferenceSpeedMS();
}
