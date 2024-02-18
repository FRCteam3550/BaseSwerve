// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.swervelib.rev;

import frc.robot.lib.swervelib.AbsoluteEncoderConfiguration;

/** Add your docs here. */
public class SparkMaxAbsoluteEncoderConfiguration implements AbsoluteEncoderConfiguration{
    public static final boolean DEFAULT_INVERTED = false;
    public final boolean inverted;

    public SparkMaxAbsoluteEncoderConfiguration() {
        this(DEFAULT_INVERTED);
    }

    public SparkMaxAbsoluteEncoderConfiguration(boolean inverted) {
        this.inverted = inverted;
    }
}
