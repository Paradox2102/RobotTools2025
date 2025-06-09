/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import robotCore.DigitalInput;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class TestLineSensor extends Command {
  private final DigitalInput m_lineSensor;
  /**
   * Creates a new TestLineSensor.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestLineSensor(DigitalInput lineSensor) {
    Logger.log("TestLineSensor", 3, "TestLineSensor()");
    m_lineSensor = lineSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestLineSensor", 2, "initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestLineSensor", -1, "execute()");
    Logger.log("TestLineSensor", 1, String.format("sensor=%b", m_lineSensor.get()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestLineSensor", 2, String.format("end(%b)", interrupted));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestLineSensor", -1, "isFinished()");
    return false;
  }
}
