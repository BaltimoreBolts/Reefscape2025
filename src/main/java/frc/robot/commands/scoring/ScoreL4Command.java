package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.elevator.ElevatorSpeedCommand;
import frc.robot.commands.elevator.StopElevatorCommand;
import frc.robot.commands.ShooterCommands.IntakeSpeedCommand;
import frc.robot.commands.ShooterCommands.ShooterAmpSpeedCommand;
import frc.robot.commands.ShooterCommands.StopShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreL4Command extends SequentialCommandGroup {
  public ScoreL4Command(ElevatorSubsystem armSubsystem, ShooterSubsystem shooterSubsystem) {
        System.out.println("scoring in L4");
    addCommands(
        new ElevatorPositionCommand(armSubsystem, ElevatorState.SCORE_L4),
        new IntakeSpeedCommand(intakeSubsystem, -1 * ShooterConstants.kIntakeSpeed + 0.0).withTimeout(0.1),
        new ShooterAmpSpeedCommand(shooterSubsystem).withTimeout(1.5),
        new IntakeSpeedCommand(intakeSubsystem, ShooterConstants.kIntakeSpeed - 0.6).withTimeout(2.0),
        new StopShooterCommand(shooterSubsystem),
        new ArmSpeedCommand(armSubsystem, () -> -0.4).withTimeout(0.4),
        new StopArmCommand(armSubsystem));
  }
}