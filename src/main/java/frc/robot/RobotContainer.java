package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.KrakenMk4iDrivetrain;

public class RobotContainer {
    private final CommandXboxController pilotGamepad = new CommandXboxController(0);
    @SuppressWarnings("unused")
    //private final KrakenMk4Drivetrain drivetrain = new KrakenMk4Drivetrain(pilotGamepad);
    //private final MaxSwerveDrivetrain drivetrain = new MaxSwerveDrivetrain(pilotGamepad);
    private final KrakenMk4iDrivetrain drivetrain = new KrakenMk4iDrivetrain(pilotGamepad);

    public RobotContainer() {       
        pilotGamepad.b().whileTrue(drivetrain.steerAllWheelsAtRestTo(Rotation2d.fromDegrees(15)));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
