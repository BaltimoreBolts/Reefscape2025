package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(ShooterSubsystem shooterSubsystem) {
        addCommands();
    }
}
