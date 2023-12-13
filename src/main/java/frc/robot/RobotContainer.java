package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MaxSwerveDrivetrain;

public class RobotContainer {
    private final CommandXboxController m_pilotGamepad = new CommandXboxController(0);
    
    private final MaxSwerveDrivetrain drivetrain = new MaxSwerveDrivetrain(m_pilotGamepad);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        m_pilotGamepad.a().onTrue(drivetrain.setBackRightAngleTo(Rotation2d.fromDegrees(0)));
        m_pilotGamepad.b().onTrue(drivetrain.setBackRightAngleTo(Rotation2d.fromDegrees(20)));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
