// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  private WPI_VictorSPX m_left1 = new WPI_VictorSPX(DriveConstants.kLeftMotor1Port);
  private WPI_VictorSPX m_left2 = new WPI_VictorSPX(DriveConstants.kLeftMotor2Port);
  private WPI_VictorSPX m_right1 = new WPI_VictorSPX(DriveConstants.kRightMotor1Port);
  private WPI_VictorSPX m_right2 = new WPI_VictorSPX(DriveConstants.kRightMotor2Port);
  private Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderChannelA, DriveConstants.kLeftEncoderChannelB);
  private Encoder m_rightEncoder = new Encoder(DriveConstants.kRightEncoderChannelA, DriveConstants.kRightEncoderChannelB);
  
  private MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_left1, m_left2);
  private MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_right1, m_right2);
  private DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDrive(double left, double right) {
    m_leftMotors.set(left);
    m_rightMotors.set(-right);
    m_drive.feed();
  }
  public void tankDriveVolts(double left, double right) {
    m_leftMotors.setVoltage(left);
    m_rightMotors.setVoltage(-right);
    m_drive.feed();
  }
  
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }
  
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }
  
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }
  
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
  
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
  
  public  double getTurnRate() {
    return -m_gyro.getRate();
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_gyro.zeroYaw();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }    
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
     
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(),m_rightEncoder.getRate());
  }
}
