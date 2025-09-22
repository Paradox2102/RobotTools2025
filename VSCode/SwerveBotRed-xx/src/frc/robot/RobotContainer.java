/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.CalibrateDrive;
import frc.robot.commands.CalibrateSteering;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TestDriveRamp;
import frc.robot.commands.TestSteeringRamp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  @SuppressWarnings("unused")
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // When using Joystick
  private final CommandJoystick m_commandJoystick = new CommandJoystick(0);
  private final Joystick m_joystick = new Joystick(0);

  // When using XBox Controller
  private final XboxController m_xbox = new XboxController(0);
  private final CommandXboxController m_commandXbox = new CommandXboxController(0);

  private final ExampleCommand m_autoCommand = null; // new ExampleCommand(m_exampleSubsystem);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Joystick control
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, () -> m_joystick.getX(),
    () -> -m_joystick.getY(), () -> m_joystick.getZ(), true));

    // XBox control
    // m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, () -> m_xbox.getLeftX(),
    //     () -> -m_xbox.getLeftY(), () -> m_xbox.getRightX(), true));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * 
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // These Commands are use to calibrate the Steering and Drive
    m_commandJoystick.button(1).toggleOnTrue(new TestSteeringRamp(m_driveSubsystem));
    m_commandJoystick.button(2).toggleOnTrue(new TestDriveRamp(m_driveSubsystem));
    m_commandJoystick.button(3).whileTrue(new CalibrateDrive(m_driveSubsystem));

    m_commandJoystick.button(7).whileTrue(new CalibrateSteering(m_driveSubsystem, 0));
    m_commandJoystick.button(8).whileTrue(new CalibrateSteering(m_driveSubsystem, 90));
    m_commandJoystick.button(9).whileTrue(new CalibrateSteering(m_driveSubsystem, 180));
    m_commandJoystick.button(10).whileTrue(new CalibrateSteering(m_driveSubsystem, 270));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
