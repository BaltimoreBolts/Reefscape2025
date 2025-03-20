package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.AlgaeScoringTarget;
import frc.robot.Constants.ElevatorConstants.ElevatorState;

public class AlgaeSubsystem extends SubsystemBase {
    private final SparkMax m_motor1 =
            new SparkMax(AlgaeConstants.kAlgaeLeftPort, MotorType.kBrushless);
    private final SparkMax m_motor2 =
            new SparkMax(AlgaeConstants.kAlgaeRightPort, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor1.getEncoder();
    private final SparkClosedLoopController m_piController1 = m_motor1.getClosedLoopController();
    SparkBaseConfig motor1Config = new SparkMaxConfig();
    SparkBaseConfig motor2Config = new SparkMaxConfig();
    private ElevatorState m_setPoint;

    // Defaults
    private AlgaeScoringTarget scoringTarget = AlgaeScoringTarget.STOW_ALGAE;
    private final double speedModifier = 0.0;

    public AlgaeSubsystem() {

        motor1Config.inverted(false).idleMode(IdleMode.kCoast);
        motor2Config.inverted(false).idleMode(IdleMode.kBrake);
        motor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.1, 0.0, 0.0);
        m_motor1.configure(
                motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor2.configure(
                motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setScoringTarget(AlgaeScoringTarget target) {
        scoringTarget = target;
    }

    public AlgaeScoringTarget getScoringTarget() {
        return scoringTarget;
    }

    public void zeroEncoders() {
        m_encoder.setPosition(ElevatorState.ZERO.getPosition());
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void setPosition(ElevatorState targetState) {
        m_setPoint = targetState;
        SmartDashboard.putNumber("algaearmsetposition", targetState.getPosition());
        m_piController1.setReference(targetState.getPosition(), ControlType.kPosition);
    }

    public void printSpeed() {
        SmartDashboard.putNumber("algaearm speed 1", m_motor1.getAppliedOutput());
    }

    public double getClosedLoopError() {
        return Math.abs(m_encoder.getPosition() - m_setPoint.getPosition());
    }

    public void stopShooter() {
        setSpeed(0.0);
    }

    // public void printTarget() {
    //     SmartDashboard.putBoolean("targetL2", scoringTarget == ScoringTarget.L2);
    //     SmartDashboard.putBoolean("targetL3", scoringTarget == ScoringTarget.L3);
    //     SmartDashboard.putBoolean("targetL4", scoringTarget == ScoringTarget.L4);
    // }

    @Override
    public void periodic() {
        // printTarget();
    }

    public void setSpeed(double speed) {
        double speedToSet = speed + speedModifier;
        if (speed == 0.0) {
            speedToSet = 0.0;
        }
        m_motor1.set(speedToSet);
        m_motor2.set(speedToSet);
    }
}
