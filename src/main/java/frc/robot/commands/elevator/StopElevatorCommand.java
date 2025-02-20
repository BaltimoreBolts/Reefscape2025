package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class StopElevatorCommand extends InstantCommand {
  private final ElevatorSubsystem elevatorSubsystem;

  public StopElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    addRequirements(elevatorSubsystem);
    this.elevatorSubsystem = elevatorSubsystem;
  }

  @Override
  public void execute() {
    elevatorSubsystem.setSpeed(0);
  }
}
