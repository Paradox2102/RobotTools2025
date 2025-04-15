/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.Encoder;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestEncoder extends Command {
  private final DriveSubsystem m_subsystem;
  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;
  private final static double k_power = 0.5;

  /**
   * Creates a new TestEncoder.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestEncoder(DriveSubsystem subsystem) {
    Logger.log("TestEncoder", 3, "TestEncoder()");

    m_subsystem = subsystem;
    m_leftEncoder = m_subsystem.getLeftEncoder();
    m_rightEncoder = m_subsystem.getRightEncoder();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestEncoder", 2, "initialize()");

    m_subsystem.setPower(0, k_power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestEncoder", -1, "execute()");
    Logger.log("TestEncoder", 1, String.format(",%d,%d", m_leftEncoder.getSpeed(), m_rightEncoder.getSpeed()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestEncoder", 2, String.format("end(%b)", interrupted));
    m_subsystem.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestEncoder", -1, "isFinished()");
    return false;
  }
}
