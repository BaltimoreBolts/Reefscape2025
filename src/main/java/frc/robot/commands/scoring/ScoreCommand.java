package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreCommand extends InstantCommand {
    private ElevatorSubsystem ElevatorSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public ScoreCommand(ElevatorSubsystem ElevatorSubsystem, ShooterSubsystem shooterSubsystem) {
        this.ElevatorSubsystem = ElevatorSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }
    
    @Override
    public void execute() {
        switch (shooterSubsystem.getScoringTarget()) {
            case L1:
                CommandScheduler.getInstance()
                        .schedule(new ScoreL1Command(ElevatorSubsystem, shooterSubsystem));
                break;
            case L2:
                CommandScheduler.getInstance()
                        .schedule(new ScoreL2Command(ElevatorSubsystem, shooterSubsystem));
                break;
            case L3:
                CommandScheduler.getInstance()
                        .schedule(new ScoreL1Command(ElevatorSubsystem, shooterSubsystem));
                break;
            case L4:
                CommandScheduler.getInstance()
                        .schedule(new ScoreL2Command(ElevatorSubsystem, shooterSubsystem));
                break;
        }
    }
}
