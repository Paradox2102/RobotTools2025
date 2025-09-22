/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Device;
import robotCore.Encoder;
import robotCore.Logger;
import robotCore.PWMMotor;
import robotCore.SmartMotor.SmartMotorMode;

public class MotorSubsystem extends SubsystemBase {
  private static final int k_motorPWMPin = Device.V3.M1_1_PWM;
  private static final int k_motorDirPin = Device.V3.M1_1_DIR;
  private static final int k_encoderPin1 = Device.V3.Q1_INT;
  private static final int k_encoderPin2 = Device.V3.Q1_DIR;


  private PWMMotor m_motor = new PWMMotor(k_motorPWMPin, k_motorDirPin);
  private Encoder m_encoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, k_encoderPin1, k_encoderPin2);
  private Encoder m_pot = new Encoder(robotCore.Encoder.EncoderType.AnalogRotational, Device.V3.A1_A, Device.V3.A1_B);

  /**
   * Creates a new DriveSubsystem.
   */
  public MotorSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");
 
    // m_motor.setInverted(true);
    // m_encoder.setInverted(true);
  }

  public void setPower(double power) {
    m_motor.setControlMode(SmartMotorMode.Power);

    m_motor.set(power);
  }

  public Encoder getEncoder() {
    return (m_encoder);
  }

  public Encoder getPot() {
    return m_pot;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }
}
