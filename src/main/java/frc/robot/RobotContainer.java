package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MaxSwerveDrivetrain;

public class RobotContainer {
    private final CommandXboxController pilotGamepad = new CommandXboxController(0);
    @SuppressWarnings("unused")
    // private final KrakenMk4Drivetrain drivetrain = new KrakenMk4Drivetrain(pilotGamepad);
    private final MaxSwerveDrivetrain drivetrain = new MaxSwerveDrivetrain(pilotGamepad);

    public Command getAutonomousCommand() {
        return null;
    }
}
