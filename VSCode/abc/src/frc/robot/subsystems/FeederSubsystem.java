/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Device;
import robotCore.DigitalCounter;
import robotCore.Encoder;
import robotCore.Logger;
import robotCore.PWMMotor;
import robotCore.Swerve;
import robotCore.Encoder.EncoderType;
import robotCore.SmartMotor.SmartMotorMode;

public class FeederSubsystem extends SubsystemBase {
  
  private final DigitalCounter m_ballCounter = new DigitalCounter(Swerve.FeederBallCounter);
  private final PWMMotor m_motor = new PWMMotor(Swerve.FeederPWM, Swerve.FeederDir, Swerve.FeederI2CAddr);
  private final Encoder m_encoder = new Encoder(EncoderType.Quadrature, Swerve.FeederEncInt, Swerve.FeederEncDir, Swerve.FeederI2CAddr);
  
  /**
   * Creates a new FeederSubsystem.
   */
  public FeederSubsystem() {
    Logger.log("FeederSubsystem", 3, "FeederSubsystem()");
    // m_motor.setInverted(true);
  }

  public void setPower(double power) {
    m_motor.setControlMode(SmartMotorMode.Power);
    m_motor.set(power);
  }

  public int getSpeed() {
    return m_encoder.getSpeed();
  }

  public void resetBallCounter() {
    m_ballCounter.reset();
  }
  
  public int getBallCount() {
    return m_ballCounter.get();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("FeederSubsystem", -1, "periodic()");
    // int speed = m_encoder.getSpeed();

    // if (Math.abs(speed) > 10)
    // {
    //   Logger.log("FeederSubsystem", 1, String.format("Speed,%d", m_encoder.getSpeed()));
    // }
  }
}
