/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;
import robotCore.Encoder;
import robotCore.Logger;

/**
 * Test the maximum drive speed.
 */
public class TestMotorSpeedCommand extends Command {
  private final MotorSubsystem m_subsystem;
  private final Encoder m_rightEncoder;
  private final double m_time;
  private final static double k_power = 1.0;
  private final Timer m_timer = new Timer();

  /**
   * Creates a new TestMotorSpeedCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestMotorSpeedCommand(MotorSubsystem subsystem, double time) {
    Logger.log("TestMotorSpeedCommand", 3, "TestMotorSpeedCommand()");

    m_subsystem = subsystem;
    m_time = time;
    m_rightEncoder = subsystem.getEncoder();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestMotorSpeedCommand", 2, "initialize()");

    m_timer.start();
    m_timer.reset();
    m_subsystem.setPower(k_power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestMotorSpeedCommand", -1, "execute()");

    Logger.log("TestMotorSpeedCommand", 0, String.format(",%d", m_rightEncoder.getSpeed()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestMotorSpeedCommand", 2, String.format("end(%b)", interrupted));

    m_subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestMotorSpeedCommand", -1, "isFinished()");
    if (m_time != 0) {
      return m_timer.get() >= m_time;
    }

    return false;
  }
}
