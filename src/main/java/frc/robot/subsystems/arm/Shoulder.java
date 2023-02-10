// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ArmConstants.ShoulderConstants;
import frc.robot.Robot;
import frc.robot.util.PidTunerSparkMaxes;

/** Add your docs here. */
public class Shoulder {
    CANSparkMax m_shoulderL_A;
    CANSparkMax m_shoulderL_B;
    CANSparkMax m_shoulderR_A;
    CANSparkMax m_shoulderR_B;
    SparkMaxAbsoluteEncoder m_AbsoluteEncoder;
    PIDTuner m_PidTuner;
    SafetyZoner m_safetyZone = new SafetyZoner(ShoulderConstants.MinAngleLimit, ShoulderConstants.MaxAngleLimit);

    public Shoulder (){
        m_shoulderL_A = new CANSparkMax(ShoulderConstants.CanIdShoulderL_A, MotorType.kBrushless);
        m_shoulderL_B = new CANSparkMax(ShoulderConstants.CanIdShoulderL_B, MotorType.kBrushless);
        m_shoulderR_A = new CANSparkMax(ShoulderConstants.CanIdShoulderR_A, MotorType.kBrushless);
        m_shoulderR_B = new CANSparkMax(ShoulderConstants.CanIdShoulderR_B, MotorType.kBrushless);
        m_shoulderR_A.setInverted(true);
        m_shoulderR_B.setInverted(true);
        m_AbsoluteEncoder = m_shoulderL_A.getAbsoluteEncoder(Type.kDutyCycle);
        m_AbsoluteEncoder.setPositionConversionFactor(ShoulderConstants.AbsoluteAngleConversionFactor);
        m_AbsoluteEncoder.setZeroOffset(ShoulderConstants.AngleZeroOffset);
        m_PidTuner = new PidTunerSparkMaxes("ShoulderPID", true, 0.0001, 0, 0, m_shoulderL_A, m_shoulderL_B, m_shoulderR_A, m_shoulderR_B);
        initializeIntegratedHallEncoder(m_shoulderL_A, getRotation().getDegrees());
        initializeIntegratedHallEncoder(m_shoulderL_B, getRotation().getDegrees());
        initializeIntegratedHallEncoder(m_shoulderR_A, getRotation().getDegrees());
        initializeIntegratedHallEncoder(m_shoulderR_B, getRotation().getDegrees());
        Robot.logManager.addNumber("Shoulder/Shoulder_rotation", () -> getRotation().getDegrees());
    }

    public Rotation2d getRotation(){
        return Rotation2d.fromDegrees(m_AbsoluteEncoder.getPosition());
    }

    public void setTargetAngle(Rotation2d angle){
        var safeAngle = m_safetyZone.getSafeValue(angle.getDegrees());
        m_shoulderL_A.getPIDController().setReference(safeAngle, ControlType.kPosition);
        m_shoulderL_B.getPIDController().setReference(safeAngle, ControlType.kPosition);
        m_shoulderR_A.getPIDController().setReference(safeAngle, ControlType.kPosition);
        m_shoulderR_B.getPIDController().setReference(safeAngle, ControlType.kPosition);
    }

    public void initializeIntegratedHallEncoder(CANSparkMax sparkMax, double currentPosition){
        var encoder = sparkMax.getEncoder();
        encoder.setPositionConversionFactor(ShoulderConstants.IntegratedEncoderAngleConversionFactor);
        encoder.setPosition(currentPosition);
    }

    public void updateSafetyZones(double extenderPosition_in, Rotation2d wristAngle) {
        // sample data - technically we want to clamp the extender and wrist based on the shoulder's position
        // So the shoulder is fine as is?
        // m_safetyZone.resetToDefaults();
    }
}
