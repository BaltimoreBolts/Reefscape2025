package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorState;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_motor1 =
            new SparkMax(ElevatorConstants.kElevatorLeft, MotorType.kBrushless);
    private final SparkMax m_motor2 =
            new SparkMax(ElevatorConstants.kElevatorRight, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor1.getEncoder();
    private final SparkClosedLoopController m_piController1 = m_motor1.getClosedLoopController();

    SparkMaxConfig motor1Config = new SparkMaxConfig();
    SparkMaxConfig motor2Config = new SparkMaxConfig();

    private ElevatorState m_setPoint;

    public ElevatorSubsystem() {
        // var motor2 = new SparkMax(ElevatorConstants.kElevatorRight, MotorType.kBrushless);

        motor1Config.inverted(false).idleMode(IdleMode.kBrake);
        motor2Config.inverted(false).follow(ElevatorConstants.kElevatorLeft).idleMode(IdleMode.kBrake);
        motor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.1, 0.0, 0.0);

        m_motor1.configure(
                motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor2.configure(
                motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        zeroEncoders();
    }

    public void zeroEncoders() {
        m_encoder.setPosition(ElevatorState.ZERO.getPosition());
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public double getClosedLoopError() {
        return Math.abs(m_encoder.getPosition() - m_setPoint.getPosition());
    }

    public void setPosition(ElevatorState targetState) {
        m_setPoint = targetState;
        SmartDashboard.putNumber("elevatorsetposition", targetState.getPosition());
        m_piController1.setReference(targetState.getPosition(), ControlType.kPosition);
    }

    public void printPosition() {
        SmartDashboard.putNumber(
                "elevator target", m_setPoint != null ? m_setPoint.getPosition() : -9999);
        SmartDashboard.putNumber("elevator position", m_encoder.getPosition());
    }

    public void printSpeed() {
        SmartDashboard.putNumber("arm speed 1", m_motor1.getAppliedOutput());
    }

    @Override
    public void periodic() {
        printPosition();
        printSpeed();
    }

    public void setSpeed(double speed) {
        // if (getPosition() >= 18 || getPosition() < 1) {
        // m_motor1.set(0);
        // }

        m_motor1.set(speed / (1.0 - ControllerConstants.kDeadzone));
    }
}
