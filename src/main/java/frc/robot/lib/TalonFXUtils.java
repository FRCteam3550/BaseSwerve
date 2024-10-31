package frc.robot.lib;

import com.ctre.phoenix6.StatusCode;

public class TalonFXUtils {
    public static final void throwIfError(StatusCode error) {
        if (error != StatusCode.OK) {
            throw new RuntimeException(String.format("Error: %s", error.name()));
        }
    }
}
