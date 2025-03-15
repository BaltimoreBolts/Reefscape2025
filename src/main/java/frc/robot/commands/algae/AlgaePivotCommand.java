package frc.robot.commands.algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaePivotCommand extends Command {
    private final AlgaeSubsystem algaeSubsystem;
    private final ElevatorState targetState;
    private int loopsWithinThreshold = 0;

    public AlgaePivotCommand(AlgaeSubsystem algaeSubsystem, ElevatorState targetState) {
        this.algaeSubsystem = algaeSubsystem;
        this.targetState = targetState;
        addRequirements(algaeSubsystem);
    }

    @Override
    public void initialize() {
        algaeSubsystem.setPosition(targetState);
        SmartDashboard.putBoolean("Arm at position", false);
    }

    @Override
    public void execute() {
        if (algaeSubsystem.getClosedLoopError() < ElevatorConstants.kToleranceRotations) {
            loopsWithinThreshold++;
        } else {
            loopsWithinThreshold = 0;
        }

        SmartDashboard.putNumber("Arm loops within threshold", loopsWithinThreshold);
        System.out.println("arm loops at position: " + loopsWithinThreshold);
        algaeSubsystem.printSpeed();
    }

    @Override
    public boolean isFinished() {
        if (loopsWithinThreshold >= 20) {
            SmartDashboard.putBoolean("Arm at position", true);
            return true;
        }
        return false;
    }
}
