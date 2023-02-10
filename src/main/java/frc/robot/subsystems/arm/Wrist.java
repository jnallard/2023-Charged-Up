// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants.WristConstants;

/** Add your docs here. */
public class Wrist {
    CANSparkMax m_wrist_A;
    SparkMaxAbsoluteEncoder m_AbsoluteEncoder;
    PIDTuner m_PidTuner;
    SafetyZoner m_safetyZone = new SafetyZoner(WristConstants.MinAngleLimit, WristConstants.MaxAngleLimit);

    public Wrist () {
        m_wrist_A = new CANSparkMax(WristConstants.CanIdWrist_A, MotorType.kBrushless);
        m_AbsoluteEncoder = m_wrist_A.getAbsoluteEncoder(Type.kDutyCycle);
        m_AbsoluteEncoder.setPositionConversionFactor(WristConstants.AbsoluteAngleConversionFactor);
        m_AbsoluteEncoder.setZeroOffset(WristConstants.AngleZeroOffset);
        m_PidTuner = new PIDTuner("WristPID", true, 0.0001, 0, 0, this::tunePID);
        initializeIntegratedHallEncoder(m_wrist_A, getRotation().getDegrees());
        Robot.logManager.addNumber("Wrist/Wrist_rotation", () -> getRotation().getDegrees());
    }

    public Rotation2d getRotation(){
        return Rotation2d.fromDegrees(m_AbsoluteEncoder.getPosition());
    }

    public void setTargetAngle(Rotation2d angle){
        var safeAngle = m_safetyZone.getSafeValue(angle.getDegrees());
        m_wrist_A.getPIDController().setReference(safeAngle, ControlType.kPosition);
    }

    public void tunePID(PIDUpdate pidUpdate){
        setPID(pidUpdate, m_wrist_A.getPIDController());
    }

    public void setPID(PIDUpdate pidUpdate, SparkMaxPIDController controller){
        controller.setP(pidUpdate.P);
        controller.setI(pidUpdate.I);
        controller.setD(pidUpdate.D);
    }

    public void initializeIntegratedHallEncoder(CANSparkMax sparkMax, double currentPosition){
        var encoder = sparkMax.getEncoder();
        encoder.setPositionConversionFactor(WristConstants.IntegratedEncoderAngleConversionFactor);
        encoder.setPosition(currentPosition);
    }

    public void updateSafetyZones(Rotation2d shoulderAngle, double extenderPosition_in) {
        var shoulderDegrees = shoulderAngle.getDegrees();
        if(extenderPosition_in > 20 && (shoulderDegrees > -135 && shoulderDegrees < -45)) {
            //sample data - if around the bottom, make sure the wrist is not out (we probably don't need to be this extreme)
            // We might actually need to say that the wrist can be either side
            m_safetyZone.exclude(-90, 90);
        }
        else if(shoulderDegrees > -105 && shoulderDegrees < -15) {
            //sample data - if around the bottom, make sure the wrist is not out (we probably don't need to be this extreme)
            // This case is saying that if the extender isn't too far out, we can have a smaller unsafe window around the shoulder angle
            m_safetyZone.exclude(-45, 45);
        }
        else {
            //sample data - otherwise, defaults are fine
            m_safetyZone.resetToDefaults();
        }
    }
}
