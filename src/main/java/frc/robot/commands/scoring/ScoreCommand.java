package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreCommand extends InstantCommand {
  private ElevatorSubsystem elevatorSubsystem;
  
  private ShooterSubsystem shooterSubsystem;

  public ScoreCommand(ElevatorSubsystem elevatorSubsystem,
      ShooterSubsystem shooterSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        
        this.shooterSubsystem = shooterSubsystem;
  }

  @Override public void execute() {
      switch (shooterSubsystem.getScoringTarget()) {
      case L2:
        CommandScheduler.getInstance().schedule(new ScoreL2Command(elevatorSubsystem, shooterSubsystem));
        break;
      case L3:
        CommandScheduler.getInstance().schedule(new ScoreL3Command(elevatorSubsystem, shooterSubsystem));
        break;
      case L4:
        CommandScheduler.getInstance().schedule(new ScoreL4Command(elevatorSubsystem, shooterSubsystem));
        break;
    }
  }
}