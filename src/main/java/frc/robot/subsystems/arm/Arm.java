// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Always owns shoulder and wrist
package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  Shoulder m_shoulder;
  Extender m_extender;
  Wrist m_wrist;
  /** Creates a new Arm. */
  public Arm() {
    m_shoulder = new Shoulder();
    m_extender = new Extender();
    m_wrist = new Wrist();
  }

  @Override
  public void periodic() {
    var shoulderAngle = m_shoulder.getRotation();
    var extenderPosition_in = m_extender.getPosition_in();
    var wristAngle = m_wrist.getRotation();

    // In order for these to work, when driving the arm, we need to continually set the reference target, so the safe values update based on current values?
    m_shoulder.updateSafetyZones(extenderPosition_in, wristAngle);
    m_extender.updateSafetyZones(shoulderAngle, wristAngle);
    m_wrist.updateSafetyZones(shoulderAngle, extenderPosition_in);
  }
}
