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
 * Calibrates the speed by setting the F, P, I and IZone values.
 */
public class CalibrateSpeedCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private final double k_speed = 0.4;

  private Encoder m_leftEncoder;
  private Encoder m_rightEncoder;

  /**
   * Creates a new TestSpeedControlCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CalibrateSpeedCommand(DriveSubsystem subsystem) {
    Logger.log("TestSpeedControlCommand", 3, "TestSpeedControlCommand()");

    m_subsystem = subsystem;
    m_leftEncoder = m_subsystem.getLeftEncoder();
    m_rightEncoder = m_subsystem.getRightEncoder();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestSpeedControlCommand", 2, "initialize()");

    Logger.log("TestSpeedControlCommand", 3, "...,Target Speed,Left Speed, Right Speed");

    m_subsystem.setSpeed(k_speed, k_speed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestSpeedControlCommand", -1, "execute()");

    Logger.log("TestSpeedControl", 3, String.format(",%.0f,%d,%d", k_speed * DriveSubsystem.k_maxSpeed,
        m_leftEncoder.getSpeed(), m_rightEncoder.getSpeed()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("TestSpeedControlCommand", 2, String.format("end(%b)", interrupted));

    m_subsystem.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("CalibrateSpeedCommand", -1, "isFinished()");
    return false;
  }
}
