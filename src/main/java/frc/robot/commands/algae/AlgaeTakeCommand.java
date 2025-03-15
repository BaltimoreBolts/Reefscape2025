package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import java.util.function.Supplier;

public class AlgaeTakeCommand extends Command {
    private final AlgaeSubsystem m_algaeSubsystem;
    private final Supplier<Double> m_speedSupplier;

    public AlgaeTakeCommand(AlgaeSubsystem algaeSubsystem, Supplier<Double> speed) {
        addRequirements(algaeSubsystem);
        m_algaeSubsystem = algaeSubsystem;
        m_speedSupplier = speed;
    }

    @Override
    public void execute() {
        double speed = Math.abs(m_speedSupplier.get()) > ControllerConstants.kDeadzone
                ? m_speedSupplier.get()
                : 0.0;
        m_algaeSubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_algaeSubsystem.setSpeed(0);
    }
}
