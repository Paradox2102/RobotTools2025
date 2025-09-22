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
import robotCore.RobotCoreBase;

/**
 * An example command that uses an example subsystem.
 */
public class TestPot extends Command {
  private final MotorSubsystem m_subsystem;
  private final Encoder m_pot;

  /**
   * Creates a new TestPot.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestPot(MotorSubsystem subsystem) {
    Logger.log("TestPot", 3, "TestPot()");

    m_subsystem = subsystem;
    m_pot = m_subsystem.getPot();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestPot", 2, "initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestPot", -1, "execute()");
    Logger.log("TestPot", 1, String.format(",%d", m_pot.getPosition()));
    RobotCoreBase.sleep(100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestPot", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestPot", -1, "isFinished()");
    return false;
  }
}
