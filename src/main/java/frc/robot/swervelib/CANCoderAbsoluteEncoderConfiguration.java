// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swervelib;

/** Add your docs here. */
public class CANCoderAbsoluteEncoderConfiguration implements AbsoluteEncoderConfiguration{
    public static final int DEFAULT_READING_UPDATE_PERIOD_MS = 100;
    public final int readingUpdatePeriodMs;

    public CANCoderAbsoluteEncoderConfiguration(int readingUpdatePeriodMs) {
        this.readingUpdatePeriodMs = readingUpdatePeriodMs;
    }

    public CANCoderAbsoluteEncoderConfiguration() {
        this(DEFAULT_READING_UPDATE_PERIOD_MS);
    }
    
}
