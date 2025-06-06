package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.DigitalInput;
import robotCore.Logger;

public class DriveToLineCommand extends Command {
    private final DriveSubsystem m_subsystem;
    private final DigitalInput m_irSensor;
    private static final double k_speed = 0.5;

public DriveToLineCommand(DriveSubsystem subsystem, DigitalInput irSensor) {
    Logger.log("DriveToLiineCommand", 3, "DriveToLineCommand()");
    m_subsystem = subsystem;
    m_irSensor = irSensor;
    addRequirements(m_subsystem);
}

 @Override
 public void initialize() {
    Logger.log("DriveToLineCommand", 2, "initialize()");
    m_subsystem.setSpeed(k_speed, k_speed);
 }

 @Override
 public void execute() {
    Logger.log("DriveToLineCommand", -1, "execute()");
 }

 @Override
 public void end(boolean interrupted) {
    Logger.log("DriveToLineCommand", 2, String.format("end(%b)", interrupted));
    m_subsystem.setPower(0, 0);
 }

 @Override
 public boolean isFinished() {
    Logger.log("DriveToLineCommand", -1, "isFinished()");
    return (m_irSensor.get());
 }
    
}