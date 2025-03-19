package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends InstantCommand {

    private final ShooterSubsystem shooterSubsystem;

    /**
     * Drive using speed inputs as a percentage output of the motor
     *
     * @param shooterSubsystem The subsystem to be used
     * @param speed Supplier of straight speed
     */
    public IntakeCommand(ShooterSubsystem shooterSubsystem) {
        addRequirements(shooterSubsystem);

        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        if (shooterSubsystem.getFirstDigitalInput() == false) {
            new ShooterSpeedCommand(shooterSubsystem, 0.3).withTimeout(1.0);
        } else if (shooterSubsystem.getFirstDigitalInput() == true) {
            new ShooterSpeedCommand(shooterSubsystem, 0.1).withTimeout(1.0);
        }
    }
}
