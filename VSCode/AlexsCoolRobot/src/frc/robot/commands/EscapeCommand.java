package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import robotCore.DigitalInput;
import robotCore.Logger;

public class EscapeCommand extends Command {
    private enum State {
        DriveForward, BackUp, Turn
    }

    private final DriveSubsystem m_subsystem;
    private final DigitalInput m_irSensor;
    private State m_state;
    private static final double k_driveSpeed = 0.4;
    private final Timer m_timer = new Timer();
    private static final double k_backingTime = 0.5;
    private static final double k_turnSpeed = 0.4;
    private static final double k_minTurnTime = 0.5;
    private Random m_random = new Random(System.currentTimeMillis());
    private double m_turnTime;

    public EscapeCommand(DriveSubsystem subsystem, DigitalInput irSensor) {
        Logger.log("EscapeCommand", 3, "EscapeCommand()");
        m_subsystem = subsystem;
        m_irSensor = irSensor;

        m_timer.start();
        addRequirements(m_subsystem);
    }

    private void DriveForward() {
        Logger.log("escapeCommand", 2, "DriveForward()");
        m_state = State.DriveForward;
        m_subsystem.setSpeed(k_driveSpeed, k_driveSpeed);

    }

    protected void Turning() {
        if (m_timer.get() >= m_turnTime) {
            DriveForward();
        }
    }

    @Override
    public void initialize() {
        Logger.log("EscapeCommand", 2, "initialize()");
        DriveForward();

    }

    private void DrivingForward() {
        if (m_irSensor.get()) {
            Logger.log("EscapeCommand", 2, "BackUp");
            m_state = State.BackUp;
            m_subsystem.setSpeed(-k_driveSpeed, -k_driveSpeed);
            m_timer.reset();
        }
    }

@Override
public void execute() {
Logger.log("EscapeCommand", -1, "execute()");
switch (m_state) {
    case DriveForward:
    DrivingForward();
    break;

    case BackUp:
    Backing();
    break;

    case Turn:
    Turning();
    break;
}


}

private void Backing() {
    if (m_timer.get() >= k_backingTime) {
        Logger.log("EscapeCommand", 2, "Turn");
        m_state = State.Turn;
        if (m_random.nextInt(2) ==1) {
            m_subsystem.setSpeed(k_turnSpeed, k_turnSpeed);
        } else {
            m_subsystem.setSpeed(-k_turnSpeed, k_turnSpeed);
        }
        m_turnTime = k_minTurnTime + m_random.nextDouble();
        m_timer.reset();
    }
}

    @Override
    public void end(boolean interrupted) {
        Logger.log("EscapeCommand", 2, String.format("end(%b)", interrupted));

    }

    @Override
    public boolean isFinished() {
        Logger.log("EscapeCommand", -1, "isFinished()");
        return false;

    }
}
