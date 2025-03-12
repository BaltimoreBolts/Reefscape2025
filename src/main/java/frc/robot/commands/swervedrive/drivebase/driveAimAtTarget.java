// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class driveAimAtTarget extends Command {
  private final SwerveSubsystem SwerveSub;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;
  private double heading;
  private double lastGoodHeading;
  /** Creates a new driveAimAtTarget. */
  public driveAimAtTarget(SwerveSubsystem s_SwerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {
    this.SwerveSub = s_SwerveSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;
    lastGoodHeading = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //System.out.println("Target is: " + heading);
    //SwerveSub.driveCommand(translationX, translationY, heading);


    SwerveSub.getSwerveDrive().drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * SwerveSub.getSwerveDrive().getMaximumChassisVelocity(),
    Math.pow(translationY.getAsDouble(), 3) * SwerveSub.getSwerveDrive().getMaximumChassisVelocity()),
    heading * SwerveSub.getSwerveDrive().getMaximumChassisAngularVelocity(),
true,
false);
  }

  // Called once the command ends or is interrupted.


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return !SwerveSub.isValidVisionTarget();
    return false;
  }
}
