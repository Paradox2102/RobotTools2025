/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Encoder;
import robotCore.Logger;
import robotCore.Minibot;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class DriveSubsystem extends SubsystemBase {
  // New Motors
  // public static final double k_minPowerLeft = 0.23;
  // public static final double k_minPowerRight = 0.21;
  // public static final double k_maxSpeed = 480;

  // private static final double k_Fleft = 1.0 / k_maxSpeed;
  // private static final double k_Fright = 0.91 / k_maxSpeed;
  // private static final double k_P = 0.003; // Use the correct number for your robot
  // private static final double k_I = 0.0005;
  // private static final double k_IZone = 50;

  // Old Motors
  public static final double k_minPowerLeft = 0.25;
  public static final double k_minPowerRight = 0.25;
  public static final double k_maxSpeed = 1500;

  private static final double k_Fleft = 0.96 / k_maxSpeed;
  private static final double k_Fright = 0.94 / k_maxSpeed;
  private static final double k_P = 0.001; // Use the correct number for your robot
  private static final double k_I = 0.0001;
  private static final double k_IZone = 100;


  private PWMMotor m_leftMotor = new PWMMotor(Minibot.LeftMotorPWM, Minibot.LeftMotorDir);
  private PWMMotor m_rightMotor = new PWMMotor(Minibot.RightMotorPWM, Minibot.RightMotorDir);
  private Encoder m_leftEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, Minibot.LeftEncoderInt, Minibot.LeftEncoderDir);
  private Encoder m_rightEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, Minibot.RightEncoderInt, Minibot.RightEncoderDir);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");

    m_leftMotor.setFeedbackDevice(m_leftEncoder);
    m_rightMotor.setFeedbackDevice(m_rightEncoder);
    m_leftMotor.setMaxSpeed(k_maxSpeed);
    m_rightMotor.setMaxSpeed(k_maxSpeed);

    m_leftMotor.setMinPower(k_minPowerLeft);
    m_rightMotor.setMinPower(k_minPowerRight);
    m_leftMotor.setFTerm(k_Fleft);
    m_rightMotor.setFTerm(k_Fright);
    m_leftMotor.setPTerm(k_P);
    m_rightMotor.setPTerm(k_P);
    m_leftMotor.setITerm(k_I);
    m_rightMotor.setITerm(k_I);
    m_leftMotor.setIZone(k_IZone);
    m_rightMotor.setIZone(k_IZone);

    m_rightEncoder.setInverted(true);
  }

  public void setPower(double leftPower, double rightPower) {
    // Logger.log("DriveSubsystem", 1, String.format("setPower(%f,%f)", leftPower,
    // rightPower));
    m_leftMotor.setControlMode(SmartMotorMode.Power);
    m_rightMotor.setControlMode(SmartMotorMode.Power);

    m_rightMotor.set(rightPower);
    m_leftMotor.set(leftPower);
  }
  

  public void setSpeed(double leftSpeed, double rightSpeed) {
    m_leftMotor.setControlMode(SmartMotorMode.Speed);
    m_rightMotor.setControlMode(SmartMotorMode.Speed);

    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
  }

  public Encoder getLeftEncoder() {
    return (m_leftEncoder);
  }

  public Encoder getRightEncoder() {
    return (m_rightEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }
}
