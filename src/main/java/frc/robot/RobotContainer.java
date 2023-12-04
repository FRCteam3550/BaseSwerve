package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private final CommandXboxController m_pilotGamepad = new CommandXboxController(0);
    
    private final Drivetrain m_drivetrain = new Drivetrain(m_pilotGamepad);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
