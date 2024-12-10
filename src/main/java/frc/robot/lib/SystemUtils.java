package frc.robot.lib;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SystemUtils {
    private static final long WAIT_TIME_MS = 10;

    public static final long waitUntil(String reason, long maxWaitMs, BooleanSupplier conditionToBeTrue) {
        long totalWaitTimeMs = 0;
    
        while (!conditionToBeTrue.getAsBoolean() && totalWaitTimeMs < maxWaitMs) {
            waitFor(WAIT_TIME_MS);
            totalWaitTimeMs += WAIT_TIME_MS;
        }
    
        if (!conditionToBeTrue.getAsBoolean()) {
            throw new RuntimeException("Waited too long for: " + reason);
        }
    
        return totalWaitTimeMs;
    }

    public static final long waitEqual(String reason, long maxWaitMs, DoubleSupplier valueSupplier, double constantToCompareWith, double tolerance) {
        long totalWaitTimeMs = 0;
        double valueDifference = Math.abs(constantToCompareWith - valueSupplier.getAsDouble());
        while (valueDifference > tolerance && totalWaitTimeMs < maxWaitMs) {
            waitFor(WAIT_TIME_MS);
            totalWaitTimeMs += WAIT_TIME_MS;
            valueDifference = Math.abs(constantToCompareWith - valueSupplier.getAsDouble());
        }
    
        if (valueDifference > tolerance) {
            throw new RuntimeException(String.format(
                "Waited too long for %s : %f should be equal to %f", 
                reason, 
                valueSupplier.getAsDouble(), 
                constantToCompareWith
            ));
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
