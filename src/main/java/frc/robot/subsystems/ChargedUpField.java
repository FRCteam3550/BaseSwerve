package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.lib.FMSUtils;

/**
 * Donne accès aux coordonnées importantes de certains éléments du terrain.
 * 
 * Document de calcul des coordonnées: https://docs.google.com/presentation/d/1mX9c9Sw2toBV61v2YBbtUmS6ajEZyHKFkF6gAtpS6FU
 */
public class ChargedUpField {
    private static final Rotation2d TOWARDS_BLUE = Rotation2d.fromDegrees(180);
    private static final Rotation2d TOWARDS_RED = Rotation2d.fromDegrees(0);

    private Optional<Alliance> forceAlliance;
    private final Field2d m_fieldWidget = new Field2d();

    public ChargedUpField() {
        this(Optional.empty());
    }

    public ChargedUpField(Alliance alliance) {
        this(Optional.of(alliance));
    }

    private ChargedUpField(Optional<Alliance> forceAlliance) {
        this.forceAlliance = forceAlliance;
    }

    public void switchColorAlliance() {
        if (isBlueAlliance()) {
            forceAlliance = Optional.of(Alliance.Red);
        } else {
            forceAlliance = Optional.of(Alliance.Blue);
        }
    }

    /**
     * Met à jour la position du robot dans le widget du terrain.
     */
    public void updateRobotPose(Pose2d robotPoseM) {
        m_fieldWidget.setRobotPose(robotPoseM);
    }

    /**
     * Mets à jour la trajectoire affichée dans le widget du terrain.
     */
    public void displayTrajectory(Trajectory traj) {
        m_fieldWidget.getObject("Trajectory").setTrajectory(traj);
    }

    private boolean isBlueAlliance() {
        return forceAlliance.isPresent() ? forceAlliance.get() == Alliance.Blue : FMSUtils.currentAlliance() == Alliance.Blue;
    }

    public boolean isAllianceColorBlue() {
        return isBlueAlliance();
    }

    public double gridRobotXDirection() {
        return isBlueAlliance() ? -1 : 1;
    }

    /**
     * The orientation the robot should be facing when looking at it's alliance grid.
     */
    public Rotation2d towardSelfAlliance() {
        return isBlueAlliance() ? TOWARDS_BLUE : TOWARDS_RED;
    }

    public Rotation2d towardOppositeAlliance() {
        return isBlueAlliance() ? TOWARDS_RED : TOWARDS_BLUE;
    }
}
