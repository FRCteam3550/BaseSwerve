package frc.robot.lib;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;

public class SparkMaxUtils {
    private static final Map<Integer, SparkMax> sparkMaxes = new HashMap<>();

    public static final void throwIfError(REVLibError error) {
        if (REVLibError.kError.equals(error)) {
            throw new RuntimeException(String.format("Error: %s", error.name()));
        }
    }

    public static SparkMax getController(int sparkMaxCanId) {
        if (sparkMaxes.containsKey(sparkMaxCanId)) {
            return sparkMaxes.get(sparkMaxCanId);
        }
    
        var sparkMax = new SparkMax(sparkMaxCanId, SparkLowLevel.MotorType.kBrushless);
        
        sparkMaxes.put(sparkMaxCanId, sparkMax);
        return sparkMax;
    }
}
