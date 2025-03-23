package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeIntakeCommand extends Command {
    private final AlgaeSubsystem algaeSubsystem; 
    private final double speed;

    /**
     * Drive using speed inputs as a percentage output of the motor
     *
     * @param shooterSubsystem The subsystem to be used
     * @param speed Supplier of straight speed
     */
    public AlgaeIntakeCommand(AlgaeSubsystem algaeSubsystem, double speed) {
        addRequirements(algaeSubsystem);

        this.algaeSubsystem = algaeSubsystem;
        this.speed = speed;
    }

    @Override
    public void execute() {
        algaeSubsystem.setSpeed(speed);
    }
}
