package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;

public class ElevatorSpeedCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final Supplier<Double> m_speedSupplier;

    public ElevatorSpeedCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Double> speed) {
        addRequirements(elevatorSubsystem);
        m_elevatorSubsystem = elevatorSubsystem;
        m_speedSupplier = speed;
    }

    @Override
    public void execute() {
        double speed = Math.abs(m_speedSupplier.get()) > ControllerConstants.kDeadzone
                ? m_speedSupplier.get()
                : 0.0;
        m_elevatorSubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.setSpeed(0);
    }
}
