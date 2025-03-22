package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPositionCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final ElevatorState targetState;
    private int loopsWithinThreshold = 0;

    public ElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, ElevatorState targetState) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetState = targetState;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPosition(targetState);
        SmartDashboard.putBoolean("Arm at position", false);
    }

    @Override
    public void execute() {
        // if (elevatorSubsystem.getClosedLoopError() < ElevatorConstants.kToleranceRotations) {
        //     loopsWithinThreshold++;
        //     System.out.println("error" + elevatorSubsystem.getClosedLoopError());
        // } else {
        //     loopsWithinThreshold = 0;
        // }

        // if (elevatorSubsystem.getPosition() < targetState.getPosition()) {
        //    loopsWithinThreshold++;
        // } else {
        //    loopsWithinThreshold = 0;
        // }
        // SmartDashboard.putNumber("Arm loops within threshold", loopsWithinThreshold);
        // System.out.println("getpos" + elevatorSubsystem.getPosition());
        // System.out.println("gettarget" +targetState.getPosition());
        // System.out.println("error" + elevatorSubsystem.getClosedLoopError());
        // elevatorSubsystem.printSpeed();
    }

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
