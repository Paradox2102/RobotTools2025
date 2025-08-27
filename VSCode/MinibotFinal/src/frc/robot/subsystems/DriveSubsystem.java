/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Logger;
import robotCore.Minibot;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;
import robotCore.Encoder;
import robotCore.Encoder.EncoderType;

public class DriveSubsystem extends SubsystemBase {
  private PWMMotor m_leftMotor = new PWMMotor(Minibot.LeftMotorPWM, Minibot.LeftMotorDir);
  private PWMMotor m_rightMotor = new PWMMotor(Minibot.RightMotorPWM, Minibot.RightMotorDir);
  private Encoder m_leftEncoder = new Encoder(EncoderType.Quadrature, Minibot.LeftEncoderInt, Minibot.LeftEncoderDir);
  private Encoder m_rightEncoder = new Encoder(EncoderType.Quadrature, Minibot.RightEncoderInt,
      Minibot.RightEncoderDir);

  public static final double k_maxSpeed = 1480; // Use the correct number for your robot!
  public static final double k_minLeftPower = 0.38; // Use the correct number for your robot!
  public static final double k_minRightPower = 0.36; // Use the correct number for your robot!
  private static final double k_Fleft = 1.05 / k_maxSpeed; // Use the correct number for your robot!
  private static final double k_Fright = 1.02 / k_maxSpeed; // Use the correct number for your robot!
  private static final double k_P = 0.001;
  private static final double k_I = 0.002;
  private static final double k_IZone = 50;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");

    m_rightEncoder.setInverted(true);

    m_leftMotor.setMinPower(k_minLeftPower);
    m_rightMotor.setMinPower(k_minRightPower);
    m_leftMotor.setFTerm(k_Fleft);
    m_rightMotor.setFTerm(k_Fright);
    m_leftMotor.setFeedbackDevice(m_leftEncoder);
    m_rightMotor.setFeedbackDevice(m_rightEncoder);
    m_leftMotor.setMaxSpeed(k_maxSpeed);
    m_rightMotor.setMaxSpeed(k_maxSpeed);
    m_leftMotor.setPTerm(k_P);
    m_rightMotor.setPTerm(k_P);
    m_leftMotor.setITerm(k_I);
    m_rightMotor.setITerm(k_I);
    m_leftMotor.setIZone(k_IZone);
    m_rightMotor.setIZone(k_IZone);
  }

  public void setPower(double leftPower, double rightPower) {
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

  public void stop() {
    setPower(0, 0);
  }

  // Return the left encoder value
  public Encoder getLeftEncoder() {
    return (m_leftEncoder);
  }

  // Return the right encoder value
  public Encoder getRightEncoder() {
    return (m_rightEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }
}
