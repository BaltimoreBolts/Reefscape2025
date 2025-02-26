package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ScoringTarget;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_motor1 = new SparkMax(ShooterConstants.kShooterLeftPort, MotorType.kBrushless);
  private final SparkMax m_motor2 = new SparkMax(ShooterConstants.kShooterRightPort, MotorType.kBrushless);
  private final SparkClosedLoopController m_pidController1 = m_motor1.getClosedLoopController();
  private final SparkClosedLoopController m_pidController2 = m_motor2.getClosedLoopController();

  SparkBaseConfig motor1Config = new SparkMaxConfig();
  SparkBaseConfig motor2Config = new SparkMaxConfig(); 

  private ScoringTarget scoringTarget = ScoringTarget.L4;
  
  private double speedModifier = 0.0;

  public ShooterSubsystem() {

    motor1Config
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    motor2Config
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    motor1Config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0, 0.0, 0.0);
    motor2Config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0, 0.0, 0.0);

  }

  public void setScoringTarget(ScoringTarget target) {
    scoringTarget = target;
  }

  public ScoringTarget getScoringTarget() {
    return scoringTarget;
  }
  
  public void stopShooter() {
    setSpeed(0);
  }
  public void printTarget() {
    SmartDashboard.putBoolean("targetL2", scoringTarget == ScoringTarget.L2);
    SmartDashboard.putBoolean("targetL3", scoringTarget == ScoringTarget.L3);
    SmartDashboard.putBoolean("targetL4", scoringTarget == ScoringTarget.L4);
  }
  public void periodic() {
    printTarget();
  }

  public void setSpeed(double speed) {
    double speedToSet = speed + speedModifier;
    if (speed == 0) { speedToSet = 0; }
    m_motor1.set(speedToSet);
    m_motor2.set(speedToSet);
  }

  // public void setSpeed(double topSpeed, double bottomSpeed) {
  //       double speedToSetTop = topSpeed + speedModifier;
  //   if (topSpeed == 0) { speedToSetTop = 0; }
  //       double speedToSetBottom = bottomSpeed + speedModifier;
  //   if (bottomSpeed == 0) { speedToSetBottom = 0; }
  //     m_motor1.set(sparkMax.getAppliedOutput(), speedToSetTop);
  //     m_motor2.set(sparkMax.getAppliedOutput(), speedToSetBottom);
      
  // }

}