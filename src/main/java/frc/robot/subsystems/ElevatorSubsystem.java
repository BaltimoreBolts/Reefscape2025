package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.Constants.ControllerConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax m_motor1 = new SparkMax(ElevatorConstants.kElevatorLeft, MotorType.kBrushless);
  private final SparkMax m_motor2 = new SparkMax(ElevatorConstants.kElevatorRight, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor1.getEncoder();
  private final SparkClosedLoopController m_piController1 = m_motor1.getClosedLoopController();
  private final SparkClosedLoopController m_piController2 = m_motor2.getClosedLoopController();

  SparkMaxConfig motor1Config = new SparkMaxConfig();
  SparkMaxConfig motor2Config = new SparkMaxConfig(); 

  private ElevatorState m_setPoint; 

  public ElevatorSubsystem() {
    
    motor1Config
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    motor2Config
      .inverted(false)
      .follow(ElevatorConstants.kElevatorLeft)
      .idleMode(IdleMode.kBrake);
    motor1Config.closedLoop 
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0, 0.0, 0.0);
    motor2Config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0, 0.0, 0.0);

    //TODO IDK how to make motors follow
    //TODO IDK how the new configuring works in terms of the old burning
    m_motor1.configure(motor1Config, null, null);
    m_motor2.configure(motor2Config, null, null);

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
    m_piController2.setReference(targetState.getPosition(), ControlType.kPosition);

  }

  public void printPosition() {
    SmartDashboard.putNumber("elevator target", m_setPoint != null ? m_setPoint.getPosition() : -9999);
    SmartDashboard.putNumber("elevator position", m_encoder.getPosition());
  }

  public void printSpeed() {
    SmartDashboard.putNumber("arm speed 1", m_motor1.getAppliedOutput());
    SmartDashboard.putNumber("arm speed 2", m_motor2.getAppliedOutput());
  }

  public void periodic() {
    printPosition();
    printSpeed();
  }

  public void setSpeed(double speed) {
    // if (getPosition() >= 18 || getPosition() < 1) {
      // m_motor1.set(0);
    // }

    m_motor1.set(speed / (1.0 - OperatorConstants.DEADBAND));
  }
  
}