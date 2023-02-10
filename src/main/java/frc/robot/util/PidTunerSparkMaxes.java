// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

/** Creates a PIDTuner for SparkMaxes running in parallel. */
public class PidTunerSparkMaxes extends PIDTuner {
    public PidTunerSparkMaxes(
        String componentName,
        boolean tuningEnabled,
        double defaultP,
        double defaultI,
        double defaultD,
        CANSparkMax... sparkMaxes
    ) {
        super(
            componentName,
            tuningEnabled,
            defaultP,
            defaultI,
            defaultD,
            (PIDUpdate pidUpdate) -> {
                for (var sparkMax : sparkMaxes) {
                    var pid = sparkMax.getPIDController();
                    pid.setP(pidUpdate.P);
                    pid.setI(pidUpdate.I);
                    pid.setD(pidUpdate.D);
                }
            }
        );
    }
}
