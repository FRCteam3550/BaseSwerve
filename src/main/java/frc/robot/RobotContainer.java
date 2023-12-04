package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
    private final XboxController m_pilotGamepad = new XboxController(0);
    
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(m_pilotGamepad);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
