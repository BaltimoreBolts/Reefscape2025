package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSpeedCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private Supplier<Double> speedSupplier;

  public ElevatorSpeedCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Double> speed) {
    addRequirements(elevatorSubsystem);
    this.elevatorSubsystem = elevatorSubsystem;
    this.speedSupplier = speed;
  }

  @Override
  public void execute() {
    // operator override
    // double speed = Math.abs(speedSupplier.get()) > ControllerConstants.kDeadzone
    //     ? speedSupplier.get()
    //     : 0.0;
    //elevatorSubsystem.setSpeed(speedSupplier);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setSpeed(0);
  }
}
