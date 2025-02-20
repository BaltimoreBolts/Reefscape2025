package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorZeroPositionCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;

  public ElevatorZeroPositionCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    elevatorSubsystem.zeroEncoders();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return true;
  }

}
