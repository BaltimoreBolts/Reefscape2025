package frc.robot.commands.algae; 

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaePivotCommand extends Command {
    private final AlgaeSubsystem algaeSubsystem; 
    private final Supplier<Double> speedSupplier; 
        public AlgaePivotCommand(AlgaeSubsystem algaeSubsystem, Supplier<Double> speed) {
            addRequirements(algaeSubsystem);
            this.algaeSubsystem = algaeSubsystem; 
            this.speedSupplier = speed; 
        }
    @Override
    public void execute() {
        double speed = Math.abs(speedSupplier.get()) > ControllerConstants.kDeadzone
            ? speedSupplier.get()
            : 0.0; 

        algaeSubsystem.setSpeed(speed);
    }
    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setSpeed(0); 
    }
}