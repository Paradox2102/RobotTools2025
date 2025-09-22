/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import robotCore.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class SpinupCommand extends Command {
  private final ShooterSubsystem m_subsystem;
  private final double m_speed = 0.95;

  /**
   * Creates a new SpinupCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SpinupCommand(ShooterSubsystem subsystem) {
    Logger.log("SpinupCommand", 3, "SpinupCommand()");

    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SpinupCommand", 2, "initialize()");
    m_subsystem.setSpeed(m_speed);
    // m_subsystem.setPower(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("SpinupCommand", -1, "execute()");
    // Logger.log("SpinupCommand", 1, String.format(",%f,%d", m_speed * ShooterSubsystem.k_maxSpeed, m_subsystem.getEncoder().getSpeed()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("SpinupCommand", 2, String.format("end(%b)", interrupted));
    m_subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("SpinupCommand", -1, "isFinished()");
    return false;
  }
}
