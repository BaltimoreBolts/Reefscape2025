package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ScoringTarget;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_motor1 = new SparkMax(ShooterConstants.kShooterLeftPort, MotorType.kBrushless);
  private final SparkMax m_motor2 = new SparkMax(ShooterConstants.kShooterRightPort, MotorType.kBrushless);

  private ScoringTarget scoringTarget = ScoringTarget.SPEAKER;
  
  private double speedModifier = 0.0;

  public ShooterSubsystem() {
    shooterNearMotor.configFactoryDefault();
    shooterFarMotor.configFactoryDefault();
    shooterNearMotor.setNeutralMode(NeutralMode.Brake);
    shooterFarMotor.setNeutralMode(NeutralMode.Brake);

    shooterNearMotor.setInverted(ShooterConstants.kShooterNearInverted);
    shooterFarMotor.setInverted(ShooterConstants.kShooterFarInverted); // TODO: this should probably just be a follow
                                                                       // call?
    // shooterFarMotor.setInverted(InvertType.OpposeMaster);
  }

  public void setScoringTarget(ScoringTarget target) {
    scoringTarget = target;
  }

  public ScoringTarget getScoringTarget() {
    return scoringTarget;
  }
  
  public void incrementSpeedModifier() {
    speedModifier += 0.1;
  }

  public void decrementSpeedModifier() {
    speedModifier -= 0.1;
  }

  public void stopShooter() {
    setSpeed(0);
  }
  public void printTarget() {
    SmartDashboard.putBoolean("targetAmp", scoringTarget == ScoringTarget.AMP);
    SmartDashboard.putBoolean("targetSpeaker", scoringTarget == ScoringTarget.SPEAKER);
  }
  
  public void printSpeed() {
    SmartDashboard.putNumber("shooterAmpSpeed", ShooterConstants.kShooterAmpSpeed + speedModifier);
    SmartDashboard.putNumber("shooterSpeakerSpeed", ShooterConstants.kShooterSpeakerSpeed + speedModifier);
  }

  public void periodic() {
    printTarget();
    printSpeed();
  }

  public void setSpeed(double speed) {
    double speedToSet = speed + speedModifier;
    if (speed == 0) { speedToSet = 0; }
    shooterNearMotor.set(VictorSPXControlMode.PercentOutput, speedToSet);
    shooterFarMotor.set(VictorSPXControlMode.PercentOutput, speedToSet);
  }

  public void setSpeed(double topSpeed, double bottomSpeed) {
        double speedToSetTop = topSpeed + speedModifier;
    if (topSpeed == 0) { speedToSetTop = 0; }
        double speedToSetBottom = bottomSpeed + speedModifier;
    if (bottomSpeed == 0) { speedToSetBottom = 0; }
      shooterNearMotor.set(VictorSPXControlMode.PercentOutput, speedToSetTop);
      shooterFarMotor.set(VictorSPXControlMode.PercentOutput, speedToSetBottom);
      
  }

}