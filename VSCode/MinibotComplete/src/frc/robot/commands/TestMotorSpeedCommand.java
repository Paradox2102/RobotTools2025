/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Encoder;
import robotCore.Logger;

/**
 * Test the maximum drive speed.
 */
public class TestMotorSpeedCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_power;
  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  /**
   * Creates a new TestMotorSpeedCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestMotorSpeedCommand(DriveSubsystem subsystem) {
    Logger.log("TestMotorSpeedCommand", 3, "TestMotorSpeedCommand()");

    m_subsystem = subsystem;
    m_leftEncoder = subsystem.getLeftEncoder();
    m_rightEncoder = subsystem.getRightEncoder();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestMotorSpeedCommand", 2, "initialize()");

    Logger.log("TestMotorSpeedCommand", 0, ",Power,Left,Right");

    m_power = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestMotorSpeedCommand", -1, "execute()");

    Logger.log("TestMotorSpeedCommand", 0,
        String.format(",%f,%d,%d", m_power, m_leftEncoder.getSpeed(), m_rightEncoder.getSpeed()));

    m_subsystem.setPower(m_power, m_power);

    m_power += 0.0025;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestMotorSpeedCommand", 2, String.format("end(%b)", interrupted));

    m_subsystem.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestMotorSpeedCommand", -1, "isFinished()");
    return m_power > 1.3;
  }
}
