package frc.robot.commands.algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaePositionCommand extends Command {
    private final AlgaeSubsystem algaeSubsystem;
    private final ElevatorState targetState;
    private int loopsWithinThreshold = 0;

    public AlgaePositionCommand(AlgaeSubsystem algaeSubsystem, ElevatorState targetState) {
        this.algaeSubsystem = algaeSubsystem;
        this.targetState = targetState;
        addRequirements(algaeSubsystem);
    }

    @Override
    public void initialize() {
        algaeSubsystem.setPosition(targetState);
        SmartDashboard.putBoolean("Algae at position", false);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        if (loopsWithinThreshold == 0) {
            SmartDashboard.putBoolean("Arm at position", true);
            return true;
        }
        System.out.println("checking finish");

        return false;
    }
}
