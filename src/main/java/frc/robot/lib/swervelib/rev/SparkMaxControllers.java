package frc.robot.lib.swervelib.rev;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMaxControllers {
    private static final Map<Integer, CANSparkMax> controllers = new HashMap<>();

    public static CANSparkMax getController(int motorCanId) {
        if (controllers.containsKey(motorCanId)) {
            return controllers.get(motorCanId);
        }

        var newController = new CANSparkMax(motorCanId, MotorType.kBrushless);
        newController.restoreFactoryDefaults();
        controllers.put(motorCanId, newController);
        return newController;
    }
}
