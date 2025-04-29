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
import robotCore.PWMMotor;
import robotCore.Encoder.EncoderType;
import robotCore.SmartMotor.SmartMotorMode;
import robotCore.Swerve;

public class ShooterSubsystem extends SubsystemBase {  
  PWMMotor m_motor = new PWMMotor(Swerve.ShooterPWM, Swerve.ShooterDIR, Swerve.ShooterI2CAddr);
  Encoder m_encoder = new Encoder(EncoderType.Quadrature, Swerve.ShooterEncInt, Swerve.ShooterEncDir, Swerve.ShooterI2CAddr);

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    Logger.log("ShooterSubsystem", 3, "ShooterSubsystem()");
    m_encoder.setInverted(true);
  }

  public void setPower(double power) {
    m_motor.setControlMode(SmartMotorMode.Power);
    m_motor.set(power);
  }

  public double getSpeed() {
    return m_encoder.getSpeed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("ShooterSubsystem", -1, "periodic()");
  }
}
