// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.util.PidTunerSparkMaxes;

/** Add your docs here. */
public class Extender {
    CANSparkMax m_extender_A;
    SparkMaxAnalogSensor m_linearPot;
    PIDTuner m_PidTuner;
    SafetyZoner m_safetyZone = new SafetyZoner(ExtenderConstants.MinLimit_in, ExtenderConstants.MaxLimit_in);

    public Extender (){
        m_extender_A = new CANSparkMax(ExtenderConstants.CanIdExtender_A, MotorType.kBrushless);
        m_linearPot = m_extender_A.getAnalog(Mode.kAbsolute);
        m_linearPot.setPositionConversionFactor(ExtenderConstants.LinearPotConversionFactor);
        m_PidTuner = new PidTunerSparkMaxes("ExtenderPID", true, 0.0001, 0, 0, m_extender_A);
        initializeIntegratedHallEncoder(m_extender_A, m_linearPot.getPosition());
        Robot.logManager.addNumber("Extender/extension_in", () -> getPosition_in());
    }

    public double getPosition_in(){
        return m_linearPot.getPosition();
    }

    public void setTargetDistance_in(double position_in){
        var safePosition = m_safetyZone.getSafeValue(position_in);
        m_extender_A.getPIDController().setReference(safePosition, ControlType.kPosition);
    }

    public void initializeIntegratedHallEncoder(CANSparkMax sparkMax, double currentPosition){
        var encoder = sparkMax.getEncoder();
        encoder.setPositionConversionFactor(ExtenderConstants.IntegratedEncoderInchesConversionFactor);
        encoder.setPosition(currentPosition);
    }

    public void updateSafetyZones(Rotation2d shoulderAngle, Rotation2d wristAngle) {
        var shoulderDegrees = shoulderAngle.getDegrees();
        if(shoulderDegrees > -135 && shoulderDegrees < -45) {
            //sample data - if around the bottom, de-extend the arm
            m_safetyZone.excludeUp(ExtenderConstants.MinLimit_in + 2);
        }
        else {
            //sample data - otherwise, defaults are fine
            m_safetyZone.resetToDefaults();
        }

    }
}
