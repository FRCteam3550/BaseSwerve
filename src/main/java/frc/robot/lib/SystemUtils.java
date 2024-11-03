package frc.robot.lib;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;

public class SystemUtils {
    private static final long WAIT_TIME_MS = 10;

    public static final long waitUntil(long maxWaitMs, BooleanSupplier conditionToBeTrue) {
        long totalWaitTimeMs = 0;
    
        while (!conditionToBeTrue.getAsBoolean() && totalWaitTimeMs < maxWaitMs) {
            waitFor(WAIT_TIME_MS);
            totalWaitTimeMs += WAIT_TIME_MS;
        }
    
        if (!conditionToBeTrue.getAsBoolean()) {
            DriverStation.reportWarning("Waited too long in TalonFXSteerController!", false);
        }
    
        return totalWaitTimeMs;
    }
 
    public static final void waitFor(long waitTimeMs) {
        try {
            Thread.sleep(waitTimeMs);
        }
        catch(InterruptedException ie) {}
    }
}
