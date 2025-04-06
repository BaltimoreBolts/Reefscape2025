package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.shooter.ShooterSpeedCommand;
import frc.robot.commands.shooter.StopShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoL4Command extends SequentialCommandGroup {
    public AutoL4Command(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
                new ElevatorPositionCommand(elevatorSubsystem, ElevatorState.SCORE_L4).withTimeout(3.0),
                new StopShooterCommand(shooterSubsystem).withTimeout(1.0),
                Commands.print("scoring in L4")//,
                // new ShooterSpeedCommand(shooterSubsystem, -0.3).withTimeout(2.0),
                // new StopShooterCommand(shooterSubsystem).withTimeout(5.0) // ,
                // new ElevatorPositionCommand(elevatorSubsystem, ElevatorState.ZERO));
                );
    }
}
