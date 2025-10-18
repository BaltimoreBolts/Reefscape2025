package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.ScoringTarget;

public class ShooterSubsystem extends SubsystemBase {
    // private final SparkMax m_motor1 =
    //         new SparkMax(ShooterConstants.kShooterPort, MotorType.kBrushless);

    // private final DigitalInput m_firstSensor = new
    // DigitalInput(digitalInputConstants.kFirstSensor);

    // SparkBaseConfig motor1Config = new SparkMaxConfig();

    // Defaults
    // private final double speedModifier = 0.0;
    // private ScoringTarget scoringTarget = ScoringTarget.L1;

    // public ShooterSubsystem() {

    //     motor1Config.inverted(true).idleMode(IdleMode.kCoast);

    //     m_motor1.configure(
    //             motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // }

    // Methods for sensors
    // public boolean getFirstDigitalInput() {
    //     return m_firstSensor.get();
    // }

    // Scoring target methods
    public void setScoringTarget(ScoringTarget target) {
        // scoringTarget = target;
    }

    // public ScoringTarget getScoringTarget() {
    //     // return scoringTarget;
    // }

    // More shooter methods
    public void stopShooter() {
        setSpeed(0.0);
    }

    @Override
    public void periodic() {}

    public void setSpeed(double speed) {
        // double speedToSet = speed + speedModifier;
        // if (speed == 0.0) {
        //     speedToSet = 0.0;
        // }
        // m_motor1.set(speedToSet);
    }
}
