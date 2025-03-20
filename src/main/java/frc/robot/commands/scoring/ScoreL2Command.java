package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.elevator.StopElevatorCommand;
import frc.robot.commands.shooter.ShooterSpeedCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreL2Command extends SequentialCommandGroup {
    public ScoreL2Command(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
                new ElevatorPositionCommand(elevatorSubsystem, ElevatorState.SCORE_L2)
                        .alongWith(new StopShooterCommand(shooterSubsystem), Commands.print("scoring in L2"))
                        .andThen(
                            new ShooterSpeedCommand(shooterSubsystem, -0.3)
                        ).andThen(
                            new StopElevatorCommand(elevatorSubsystem)
                        )); 
    }
}
