// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/** An example command that uses an example subsystem. */
public class AbsoluteDrive extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier headingHorizontal, headingVertical;
    private boolean initRotation = false;

    /**
     * Used to drive a swerve robot in full field-centric mode. vX and vY supply translation inputs,
     * where x is torwards/away from alliance wall and y is left/right. headingHorzontal and
     * headingVertical are the Cartesian coordinates from which the robot's angle will be derived—
     * they will be converted to a polar angle, which the robot will rotate to.
     *
     * @param swerve The swerve drivebase subsystem.
     * @param vX DoubleSupplier that supplies the x-translation joystick input. Should be in the range
     *     -1 to 1 with deadband already accounted for. Positive X is away from the alliance wall.
     * @param vY DoubleSupplier that supplies the y-translation joystick input. Should be in the range
     *     -1 to 1 with deadband already accounted for. Positive Y is towards the left wall when
     *     looking through the driver station glass.
     * @param headingHorizontal DoubleSupplier that supplies the horizontal component of the robot's
     *     heading angle. In the robot coordinate system, this is along the same axis as vY. Should
     *     range from -1 to 1 with no deadband. Positive is towards the left wall when looking through
     *     the driver station glass.
     * @param headingVertical DoubleSupplier that supplies the vertical component of the robot's
     *     heading angle. In the robot coordinate system, this is along the same axis as vX. Should
     *     range from -1 to 1 with no deadband. Positive is away from the alliance wall.
     */
    public AbsoluteDrive(
            SwerveSubsystem swerve,
            DoubleSupplier vX,
            DoubleSupplier vY,
            DoubleSupplier headingHorizontal,
            DoubleSupplier headingVertical) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingHorizontal = headingHorizontal;
        this.headingVertical = headingVertical;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        initRotation = true;
    }
    //TODO: Where to implment this aiming and ranging simotaneously code (i.e., AbsoluteDrive, SwerveSubsystem, Robotcontainer)
    // double limelight_aim_proportional()
    // {    
    //     double kP = .035;
    //     double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
    //     targetingAngularVelocity *= Math.PI;
    //     targetingAngularVelocity *= -1.0;
    //     return targetingAngularVelocity;
    // }
    // double limelight_range_proportional()
    // {    
    //     double kP = .1;
    //     double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    //     targetingForwardSpeed *= 3.0;
    //     targetingForwardSpeed *= -1.0;
    //     return targetingForwardSpeed;
    // }

    // private void drive(boolean fieldRelative) {
    //     // Get the x speed. We are inverting this because Xbox controllers return
    //     // negative values when we push forward.
    //     var xSpeed =
    //         -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
    //             * 3.0;

    //     // Get the y speed or sideways/strafe speed. We are inverting this because
    //     // we want a positive value when we pull to the left. Xbox controllers
    //     // return positive values when you pull to the right by default.
    //     var ySpeed =
    //         -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
    //             * 3.0;

    //     // Get the rate of angular rotation. We are inverting this because we want a
    //     // positive value when we pull to the left (remember, CCW is positive in
    //     // mathematics). Xbox controllers return positive values when you pull to
    //     // the right by default.
    //     var rot =
    //         -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
    //             * Math.PI;

    //     // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    //     if(m_controller.getAButton())
    //     {
    //         final var rot_limelight = limelight_aim_proportional();
    //         rot = rot_limelight;

    //         final var forward_limelight = limelight_range_proportional();
    //         xSpeed = forward_limelight;

    //         //while using Limelight, turn off field-relative driving.
    //         fieldRelative = false;
    //     }
    // }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Get the desired chassis speeds based on a 2 joystick module.
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
                vX.getAsDouble(),
                vY.getAsDouble(),
                headingHorizontal.getAsDouble(),
                headingVertical.getAsDouble());

        // Prevent Movement After Auto
        if (initRotation) {
            if (headingHorizontal.getAsDouble() == 0 && headingVertical.getAsDouble() == 0) {
                // Get the curretHeading
                Rotation2d firstLoopHeading = swerve.getHeading();

                // Set the Current Heading to the desired Heading
                desiredSpeeds =
                        swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
            }
            // Dont Init Rotation Again
            initRotation = false;
        }

        // Limit velocity to prevent tippy
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(
                translation,
                swerve.getFieldVelocity(),
                swerve.getPose(),
                Constants.LOOP_TIME,
                Constants.ROBOT_MASS,
                List.of(Constants.CHASSIS),
                swerve.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // Make the robot move
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
