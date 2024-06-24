// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.pid.PIDTuner;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.logging.LogManager;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;
 // AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;
  private Field2d m_field;
  private Rotation2d m_simrotation = new Rotation2d();

  private PIDController m_XPid;
  private PIDController m_YPid;
  private PIDController m_AnglePid;
  private PIDTuner m_XPidTuner;
  private PIDTuner m_YPidTuner;
  private PIDTuner m_AnglePidTuner;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    Translation2d frontLeftTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d frontRightTranslation = new Translation2d(SwerveConstants.RobotLength_m / 2,-SwerveConstants.RobotWidth_m / 2);
    Translation2d backLeftTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, SwerveConstants.RobotWidth_m / 2);
    Translation2d backRightTranslation = new Translation2d(-SwerveConstants.RobotLength_m / 2, -SwerveConstants.RobotWidth_m / 2);
    m_frontLeft = new SwerveModule(
      frontLeftTranslation, 
      SwerveConstants.CanIdFrontLeftAngle, 
      SwerveConstants.CanIdFrontLeftVelocity, 
      22, 
      250);
    m_frontRight = new SwerveModule(
      frontRightTranslation, 
      SwerveConstants.CanIdFrontRightAngle, 
      SwerveConstants.CanIdFrontRightVelocity, 
      21, 
      -3);
    m_backLeft = new SwerveModule(
      backLeftTranslation, 
      SwerveConstants.CanIdBackLeftAngle, 
      SwerveConstants.CanIdBackLeftVelocity,
       23, 
       57);
    m_backRight = new SwerveModule(
      backRightTranslation, 
      SwerveConstants.CanIdBackRightAngle, 
      SwerveConstants.CanIdBackRightVelocity, 
      20, 
      -161);
    m_kinematics = new SwerveDriveKinematics(
        getModuleTranslations());
    m_odometry = new SwerveDriveOdometry(
        m_kinematics, getRawRotation(),
        getModulePositions());
    m_field = new Field2d();
    SmartDashboard.putData("SwerveDrive", m_field);
    m_XPid = new PIDController(1, 0, 0);
    m_YPid = new PIDController(1, 0, 0);
    m_AnglePid = new PIDController(1, 0, 0);
    m_AnglePid.enableContinuousInput(-Math.PI, Math.PI);
    m_XPidTuner = new PIDTuner("X PID Tuner", true, m_XPid);
    m_YPidTuner = new PIDTuner("Y PID Tuner", true, m_YPid);
    m_AnglePidTuner = new PIDTuner("Angel PID Tuner", true, m_AnglePid);
    Robot.logManager.addNumber("SwerveDrive/X_m", () -> m_odometry.getPoseMeters().getX());
    Robot.logManager.addNumber("SwerveDrive/Y_m", () -> m_odometry.getPoseMeters().getY());
    Robot.logManager.addNumber("SwerveDrive/Rotation_deg", () -> getRobotOnFieldRotation().getDegrees());
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    };
  }

  private Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      m_frontLeft.getTranslation(),
      m_frontRight.getTranslation(),
      m_backLeft.getTranslation(),
      m_backRight.getTranslation()
    };
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getModuleState(),
      m_frontRight.getModuleState(),
      m_backLeft.getModuleState(),
      m_backRight.getModuleState()
    };
  }

  public void move(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    m_frontLeft.setTarget(states[0]);
    m_frontRight.setTarget(states[1]);
    m_backLeft.setTarget(states[2]);
    m_backRight.setTarget(states[3]);
  }


  public void moveFieldRelative(double xMetersPerSecond, double yMetersPerSecond, double omegaRadianPerSecond){
    ChassisSpeeds speeds=ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadianPerSecond, getRobotOnFieldRotation());
    move(speeds);
  }

  public void moveRobotRelative(double xForwardSpeedMetersPerSecond, double ySidewaySpeedMetersPerSecond,
      double omegaRadianPerSecond) {
    ChassisSpeeds speeds = new ChassisSpeeds(xForwardSpeedMetersPerSecond, ySidewaySpeedMetersPerSecond,
        omegaRadianPerSecond);
    move(speeds);
  }

  public void resetPids() {
    System.out.println("resetPids");
    m_XPid.reset();
    m_YPid.reset();
    m_AnglePid.reset();
  }

  public boolean atTarget() {
    return m_XPid.atSetpoint() && m_YPid.atSetpoint() && m_AnglePid.atSetpoint();
  }

  public void setCoordinates(double x, double y, Rotation2d angle) {
    m_XPid.setSetpoint(x);
    m_YPid.setSetpoint(y);
    m_AnglePid.setSetpoint(angle.getRadians());
  }

  public double clamp(double value, double max) {
    return MathUtil.clamp(value, -1, 1) * max;
  }

  public void moveToTarget() {
    Pose2d pose = m_odometry.getPoseMeters();
    double x = clamp(m_XPid.calculate(pose.getX()), Constants.SwerveConstants.MaxRobotSpeed_mps);
    double y = clamp(m_YPid.calculate(pose.getY()), Constants.SwerveConstants.MaxRobotSpeed_mps);
    double angle = clamp(m_AnglePid.calculate(getRobotOnFieldRotation().getRadians()), Constants.SwerveConstants.MaxRobotRotation_radps);
    System.out.println(String.format("x: %s\t y: %s \t angle: %s \t pose: %s", x, y, angle, pose));
    moveFieldRelative(x, y, angle);
  }

  /**
   * Returns the raw gyro angle of the robot (or simulated version)
   * IMPORTANT: This function should only be used for updating odometry
   */
  public Rotation2d getRawRotation() {
    if (Robot.isSimulation()) {
      return m_simrotation;
    }
    // return m_gyro.getRotation2d();
    return new Rotation2d();
  }

  /**
   * Returns the robot's angle in relation to the field/odometry
   */
  public Rotation2d getRobotOnFieldRotation() {
    return m_odometry.getPoseMeters().getRotation();
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
      ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(m_frontLeft.getModuleState(), m_frontRight.getModuleState(),
          m_backLeft.getModuleState(), m_backRight.getModuleState());
      double radians = speeds.omegaRadiansPerSecond / Constants.UpdateFrequency_Hz;
      m_simrotation = m_simrotation.plus(Rotation2d.fromRadians(radians));
      SmartDashboard.putNumber("simAngle", m_simrotation.getDegrees());
    }
    Pose2d robotPose = m_odometry.update(getRawRotation(), getModulePositions());
    SmartDashboard.putNumber("SwerveDrive/X", robotPose.getX());
    SmartDashboard.putNumber("SwerveDrive/y", robotPose.getY());
    SmartDashboard.putNumber("SwerveDrive/Rotation", robotPose.getRotation().getDegrees());
    m_field.setRobotPose(robotPose);
    updateModuleOnField(m_frontLeft, robotPose, "FL");
    updateModuleOnField(m_frontRight, robotPose, "FR");
    updateModuleOnField(m_backLeft, robotPose, "BL");
    updateModuleOnField(m_backRight, robotPose, "BR");
    SmartDashboard.putNumber("rawAngle", getRawRotation().getDegrees());
    SmartDashboard.putNumber("Angle", getRobotOnFieldRotation().getDegrees());
    m_XPidTuner.tune();
    m_YPidTuner.tune();
    m_AnglePidTuner.tune();
    m_frontLeft.getModuleInfo("FL");
    m_frontRight.getModuleInfo("FR");
    m_backLeft.getModuleInfo("BL");
    m_backRight.getModuleInfo("BR");
  }

  public void resetPose(Pose2d robotPose){
    if (Robot.isSimulation()) {
      m_simrotation = robotPose.getRotation();
    }
    m_odometry.resetPosition(getRawRotation(), getModulePositions(), robotPose);
  }

  public void updateModuleOnField(SwerveModule swerveModule, Pose2d robotPose, String name) {
    Transform2d transform = new Transform2d(swerveModule.getTranslation().times(5), swerveModule.getModuleState().angle);
    Pose2d swerveModulePose = robotPose.transformBy(transform);
    m_field.getObject(name).setPose(swerveModulePose);
  }

}