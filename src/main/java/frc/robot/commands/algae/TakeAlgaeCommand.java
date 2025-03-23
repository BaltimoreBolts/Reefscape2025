package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeSubsystem;

public class TakeAlgaeCommand extends SequentialCommandGroup {
    public TakeAlgaeCommand(AlgaeSubsystem algaeSubsystem) {
        addCommands();
    }
}
