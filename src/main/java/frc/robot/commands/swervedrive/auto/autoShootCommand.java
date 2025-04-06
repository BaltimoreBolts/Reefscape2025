package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.shooter.ShooterSpeedCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class autoShootCommand extends SequentialCommandGroup {
    public autoShootCommand(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        addCommands(
                new ShooterSpeedCommand(shooterSubsystem, ShooterConstants.kShooterSpeed).withTimeout(5.0),
                new ShooterSpeedCommand(shooterSubsystem, 0).withTimeout(2),
                new ElevatorPositionCommand(elevatorSubsystem, ElevatorState.ZERO));
    }
}
