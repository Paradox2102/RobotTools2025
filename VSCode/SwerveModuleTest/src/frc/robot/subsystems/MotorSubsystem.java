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

public class MotorSubsystem extends SubsystemBase {
  private PWMMotor m_driveMotor = new PWMMotor(Device.V3.M1_2_PWM, Device.V3.M1_2_DIR);
  private Encoder m_driveEncoder = new Encoder(robotCore.Encoder.EncoderType.Quadrature, Device.V3.Q1_INT, Device.V3.Q1_DIR);
  private PWMMotor m_steeringMotor = new PWMMotor(Device.V3.M1_1_PWM, Device.V3.M1_1_DIR);
  private Encoder m_steeringEncoder = new Encoder(robotCore.Encoder.EncoderType.AnalogRotational, Device.V3.A1_A, Device.V3.A1_B);

  /**
   * Creates a new DriveSubsystem.
   */
  public MotorSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");
 
    // m_driveMotor.setInverted(true);
    // m_driveEncoder.setInverted(true);
  }

  public void setDrivePower(double power) {
    m_driveMotor.set(power);
  }

  public void setSteeringPower(double power) {
    m_steeringMotor.set(power);
  }

  public Encoder getDriveEncoder() {
    return m_driveEncoder;
  }

  public Encoder getSteeringEncoder() {
    return m_steeringEncoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }
}
