package frc.robot.lib;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class SparkMaxUtils {
    private static final Map<Integer, CANSparkMax> sparkMaxes = new HashMap<>();

    public static final void throwIfError(REVLibError error) {
        if (error != REVLibError.kOk) {
            throw new RuntimeException(String.format("Error: %s", error.name()));
        }
    }

    public static CANSparkMax getController(int sparkMaxCanId) {
        if (sparkMaxes.containsKey(sparkMaxCanId)) {
            return sparkMaxes.get(sparkMaxCanId);
        }
    
        var sparkMax = new CANSparkMax(sparkMaxCanId, CANSparkLowLevel.MotorType.kBrushless);
        sparkMax.restoreFactoryDefaults();
        sparkMaxes.put(sparkMaxCanId, sparkMax);
        return sparkMax;
    }
}
