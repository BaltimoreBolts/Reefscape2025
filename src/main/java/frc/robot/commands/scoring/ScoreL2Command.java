package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreL2Command extends SequentialCommandGroup {
    public ScoreL2Command(ElevatorSubsystem armSubsystem, ShooterSubsystem shooterSubsystem) {
        System.out.println("scoring in L2");
        addCommands(new ElevatorPositionCommand(armSubsystem, ElevatorState.SCORE_L2));
    }
}
