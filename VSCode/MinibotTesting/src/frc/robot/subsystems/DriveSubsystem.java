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
  private PWMMotor m_leftMotor = new PWMMotor(Minibot.LeftMotorPWM, Minibot.LeftMotorDir);
  private PWMMotor m_rightMotor = new PWMMotor(Minibot.RightMotorPWM, Minibot.RightMotorDir);
  private Encoder m_leftEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, Minibot.LeftEncoderInt, Minibot.LeftEncoderDir);
  private Encoder m_rightEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, Minibot.RightEncoderInt, Minibot.RightEncoderDir);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");
 
    m_rightEncoder.setInverted(true);
  }

  public void setPower(double leftPower, double rightPower) {
    m_leftMotor.setControlMode(SmartMotorMode.Power);
    m_rightMotor.setControlMode(SmartMotorMode.Power);

    m_rightMotor.set(rightPower);
    m_leftMotor.set(leftPower);
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
