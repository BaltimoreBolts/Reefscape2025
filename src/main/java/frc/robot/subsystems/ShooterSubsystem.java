package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.inputConstants.digitalInputConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax m_motor1 =
            new SparkMax(ShooterConstants.kShooterLeftPort, MotorType.kBrushless);
    private final SparkMax m_motor2 =
            new SparkMax(ShooterConstants.kShooterRightPort, MotorType.kBrushless);
    private final DigitalInput m_firstSensor = new DigitalInput(digitalInputConstants.kFirstSensor);
    private final DigitalInput m_secondSensor = new DigitalInput(digitalInputConstants.kSecondSensor);

    SparkBaseConfig motor1Config = new SparkMaxConfig();
    SparkBaseConfig motor2Config = new SparkMaxConfig();

    private final double speedModifier = 0.0;

    public ShooterSubsystem() {

        motor1Config.inverted(true).idleMode(IdleMode.kCoast);
        motor2Config.inverted(false).idleMode(IdleMode.kCoast);

        m_motor1.configure(
                motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor2.configure(
                motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean getFirstDigitalInput() {
        return m_firstSensor.get();
    }

    public boolean getSecondDigitalInput() {
        return m_secondSensor.get();
    }

    public void stopShooter() {
        setSpeed(0.0);
    }

    @Override
    public void periodic() {}

    public void setSpeed(double speed) {
        double speedToSet = speed + speedModifier;
        if (speed == 0.0) {
            speedToSet = 0.0;
        }
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
