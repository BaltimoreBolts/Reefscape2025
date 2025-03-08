// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.elevator.ElevatorSpeedCommand;
import frc.robot.commands.scoring.ScoreL1Command;
import frc.robot.commands.scoring.ScoreL2Command;
import frc.robot.commands.scoring.ScoreL3Command;
import frc.robot.commands.shooter.ShooterSpeedCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */



public class RobotContainer {
    private ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final SendableChooser<Command> autoChooser;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController driverXbox = new CommandXboxController(0);
    private final Joystick m_operatorController = new Joystick(1);
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase =
            new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
     * velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
                    drivebase.getSwerveDrive(),
                    () -> driverXbox.getLeftY() * -1,
                    () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
    SwerveInputStream driveDirectAngle = driveAngularVelocity
            .copy()
            .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
            .headingWhile(true);

    /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
    SwerveInputStream driveRobotOriented =
            driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
                    drivebase.getSwerveDrive(), () -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX())
            .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard
            .copy()
            .withControllerHeadingAxis(
                    () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                    () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
            .headingWhile(true);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Subsystem initialization

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        // Named command registration for PathPlanner Autos
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {

        // Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        // Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        // Command driveSetpointGen =
        // drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard =
                drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        // Command driveFieldOrientedAnglularVelocityKeyboard =
        //         drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        // Command driveSetpointGenKeyboard =
        //         drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        // *************************
        //    Simulation Commands
        // *************************
        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);

            driverXbox
                    .start()
                    .onTrue(
                            Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
        }

        // ************************
        //    Test Mode Commands
        // ************************
        var isTest = new Trigger(DriverStation::isTest);
        driverXbox
                .x()
                .and(isTest)
                .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        driverXbox.y().and(isTest).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
        driverXbox.start().and(isTest).onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.back().and(isTest).whileTrue(drivebase.centerModulesCommand());
        // driverXbox.leftBumper().and(isTest).onTrue(Commands.none());
        // driverXbox.rightBumper().and(isTest).onTrue(Commands.none());

        // **************************
        //    Normal Mode Commands
        // **************************

        // Driver Controls
        driverXbox.a().and(isTest.negate()).onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.x().and(isTest.negate()).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
        driverXbox
                .b()
                .and(isTest.negate())
                .whileTrue(
                        drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
        // driverXbox.start().and(isTest.negate()).whileTrue(Commands.none());
        // driverXbox.back().and(isTest.negate()).whileTrue(Commands.none());
        driverXbox
                .leftBumper()
                .and(isTest.negate())
                .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        // driverXbox.rightBumper().and(isTest.negate()).onTrue(Commands.none());

        // Operator Controls
        var operatorLeftStickY = new Trigger(() ->
                Math.abs(m_operatorController.getRawAxis(Axis.kLeftY)) > ControllerConstants.kDeadzone);
        operatorLeftStickY.whileTrue(new ElevatorSpeedCommand(
                m_elevatorSubsystem, () -> -1.0 * m_operatorController.getRawAxis(Axis.kLeftY)));

        new JoystickButton(m_operatorController, ControllerConstants.Button.kA)
                .whileTrue(new ScoreL1Command(m_elevatorSubsystem, m_shooterSubsystem));
        new JoystickButton(m_operatorController, ControllerConstants.Button.kB)
                .whileTrue(new ScoreL2Command(m_elevatorSubsystem, m_shooterSubsystem));
        new JoystickButton(m_operatorController, ControllerConstants.Button.kY)
                .whileTrue(new ScoreL3Command(m_elevatorSubsystem, m_shooterSubsystem));

        new JoystickButton(m_operatorController, ControllerConstants.Button.kX)
                .whileTrue(new ShooterSpeedCommand(m_shooterSubsystem, 0.35))
                .whileFalse(new ShooterSpeedCommand(m_shooterSubsystem, 0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
