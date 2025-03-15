package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeIntakeCommand extends SequentialCommandGroup {
    public AlgaeIntakeCommand(AlgaeSubsystem algaeSubsystem) {
        addCommands(
                new AlgaePivotCommand(algaeSubsystem, ElevatorState.STOW_ALGAE),
                new AlgaeTakeCommand(algaeSubsystem, -1.0),
                new AlgaePivotCommand(algaeSubsystem, ElevatorState.INTAKE_ALGAE));
    }
}
