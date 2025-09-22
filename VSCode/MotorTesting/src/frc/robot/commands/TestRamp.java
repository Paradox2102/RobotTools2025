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
 * Test the maximum drive speed.
 */
public class TestRamp extends Command {
  private final MotorSubsystem m_subsystem;
  private final Encoder m_encoder;
  private double m_power = 0;
  private boolean m_reverse = false;

  /**
   * Creates a new TestRamp.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestRamp(MotorSubsystem subsystem) {
    Logger.log("TestRamp", 3, "TestRamp()");

    m_subsystem = subsystem;
    m_encoder = subsystem.getEncoder();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("TestRamp", 2, "initialize()");
    m_reverse = false;
    m_power = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.log("TestRamp", -1, "execute()");

    Logger.log("TestRamp", 0, String.format(",%f,%d", m_power, m_encoder.getSpeed()));

    m_subsystem.setPower(m_power);

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
    Logger.log("TestRamp", 2, String.format("end(%b)", interrupted));

    m_subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.log("TestRamp", -1, "isFinished()");

    return m_power < -1.2;
  }
}
