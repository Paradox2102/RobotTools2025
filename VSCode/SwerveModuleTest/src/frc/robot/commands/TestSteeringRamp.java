/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;
import robotCore.Encoder;
import robotCore.Logger;

/**
 * Test the maximum sterring speed.
 */
public class TestSteeringRamp extends Command {
  private final MotorSubsystem m_subsystem;
  private final Encoder m_encoder;
  private double m_power = 0;
  private boolean m_reverse = false;

  /**
   * Creates a new TestMotorSpeedCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestSteeringRamp(MotorSubsystem subsystem) {
    Logger.log("TestMotorSpeedCommand", 3, "TestMotorSpeedCommand()");

    m_subsystem = subsystem;
    m_encoder = subsystem.getSteeringEncoder();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestMotorSpeedCommand", 2, "initialize()");

    m_power = 0;
    m_reverse = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestMotorSpeedCommand", -1, "execute()");

    Logger.log("TestMotorSpeedCommand", 0, String.format(",%f,%d", m_power, m_encoder.getPosition()));

    m_subsystem.setSteeringPower(m_power);

    if (m_reverse) {
      m_power -= 0.005;
    }
    else {
      m_power += 0.005;
    }

    if (!m_reverse && (m_power >= 1.2))
    {
      m_reverse = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestMotorSpeedCommand", 2, String.format("end(%b)", interrupted));

    m_subsystem.setSteeringPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestMotorSpeedCommand", -1, "isFinished()");

    return m_power < -0.75;
  }
}
