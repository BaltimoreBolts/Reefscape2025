package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreL4Command extends SequentialCommandGroup {
    public ScoreL4Command(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(new ElevatorPositionCommand(elevatorSubsystem, ElevatorState.SCORE_L4)
                .alongWith(new StopShooterCommand(shooterSubsystem), Commands.print("scoring in L4")));
        // .withTimeout(5.0)
        // .andThen(new ShooterSpeedCommand(shooterSubsystem, -0.3))
        // .withTimeout(1.0)
        // .andThen(new ElevatorPositionCommand(elevatorSubsystem, ElevatorState.ZERO)));
    }
}
