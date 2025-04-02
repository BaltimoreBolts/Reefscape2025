// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.DPad;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.algae.AlgaeIntakeCommand;
import frc.robot.commands.algae.AlgaePivotCommand;
import frc.robot.commands.elevator.ElevatorZeroPositionCommand;
import frc.robot.commands.scoring.BargeCommand;
import frc.robot.commands.scoring.ScoreCommand;
import frc.robot.commands.scoring.ScoreL1Command;
import frc.robot.commands.scoring.ScoreL2Command;
import frc.robot.commands.scoring.ScoreL3Command;
import frc.robot.commands.scoring.ScoreL4Command;
import frc.robot.commands.scoring.ZeroCommand;
import frc.robot.commands.shooter.ShooterSpeedCommand;
// import frc.robot.commands.shooter.IntakeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
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
    // Subsystem instantiation
    private ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

    // Command instantiation
    private ScoreL1Command m_scoreL1Command =
            new ScoreL1Command(m_elevatorSubsystem, m_shooterSubsystem);
    private ScoreL2Command m_scoreL2Command =
            new ScoreL2Command(m_elevatorSubsystem, m_shooterSubsystem);
    private ScoreL3Command m_scoreL3Command =
            new ScoreL3Command(m_elevatorSubsystem, m_shooterSubsystem);
    private ScoreL4Command m_scoreL4Command =
            new ScoreL4Command(m_elevatorSubsystem, m_shooterSubsystem);
    private final SendableChooser<Command> autoChooser;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController driverController = new CommandXboxController(0);
    private final Joystick operatorController = new Joystick(1);
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase =
            new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
     * velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
                    drivebase.getSwerveDrive(),
                    () -> driverController.getLeftY() * -1,
                    () -> driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverController.getRightX() * -1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
    SwerveInputStream driveDirectAngle = driveAngularVelocity
            .copy()
            .withControllerHeadingAxis(driverController::getRightX, driverController::getRightY)
            .headingWhile(true);

    /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
    SwerveInputStream driveRobotOriented =
            driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
                    drivebase.getSwerveDrive(),
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX())
            .withControllerRotationAxis(() -> driverController.getRawAxis(2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard
            .copy()
            .withControllerHeadingAxis(
                    () -> Math.sin(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                    () -> Math.cos(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
            .headingWhile(true);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Subsystem initialization

        // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        // Named command registration for PathPlanner Autos
        NamedCommands.registerCommand("score L1", m_scoreL1Command);
        NamedCommands.registerCommand("score L2", m_scoreL2Command);
        NamedCommands.registerCommand("score L3", m_scoreL3Command);
        NamedCommands.registerCommand("score L4", m_scoreL4Command);
        //      NamedCommands.registerCommand("take algae 1", m_takealgae1Command);
        //      NamedCommands.registerCommand("take algae 2", m_takealgae2Command);
        //      NamedCommands.registerCommand("score barge", m_bargeCommand);
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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

            driverController
                    .start()
                    .onTrue(
                            Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
        }

        // ************************
        //    Test Mode Commands
        // ************************
        var isTest = new Trigger(DriverStation::isTest);
        driverController
                .x()
                .and(isTest)
                .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        driverController.y().and(isTest).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
        driverController.start().and(isTest).onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverController.back().and(isTest).whileTrue(drivebase.centerModulesCommand());
        // driverController.leftBumper().and(isTest).onTrue(Commands.none());
        // driverController.rightBumper().and(isTest).onTrue(Commands.none());

        // **************************
        //    Normal Mode Commands
        // **************************

        /*
         * =========================================
         * | DRIVER CONTROLS |
         * =========================================
         */

        // Scores score state set by operator
        driverController.a().onTrue(new ScoreCommand(m_elevatorSubsystem, m_shooterSubsystem));

        // Zero gyro
        driverController.povLeft().and(isTest.negate()).onTrue((Commands.runOnce(drivebase::zeroGyro)));

        // driverController
        //         .x()
        //         .and(isTest.negate())
        //         .onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
        // driverController
        //         .b()
        //         .and(isTest.negate())
        //         .whileTrue(
        //                 drivebase.driveToPose(new Pose2d(new Translation2d(4, 4),
        // Rotation2d.fromDegrees(0))));
        // driverController
        //         .povRight()
        //         .and(isTest.negate())
        //         .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

        // Auto-aligns
        driverController
                .rightBumper()
                .onTrue(new AlignToReefTagRelative(true, drivebase).withTimeout(7));
        driverController
                .leftBumper()
                .onTrue(new AlignToReefTagRelative(false, drivebase).withTimeout(7));

        // // Intake and outtake
        driverController
                .rightTrigger()
                .whileTrue(new ShooterSpeedCommand(m_shooterSubsystem, -0.35))
                .whileFalse(new ShooterSpeedCommand(m_shooterSubsystem, 0));
        driverController
                .x()
                .whileTrue(new ShooterSpeedCommand(m_shooterSubsystem, -0.6))
                .whileFalse(new ShooterSpeedCommand(m_shooterSubsystem, 0));

        // Adjust coral in end effector
        driverController
                .leftTrigger()
                .whileTrue(new ShooterSpeedCommand(m_shooterSubsystem, 0.35))
                .whileFalse(new ShooterSpeedCommand(m_shooterSubsystem, 0));

        // TODO: want to change to driver and replace with current intake
        // Intake coral
        // new JoystickButton(operatorController, ControllerConstants.Button.kLeftBumper)
        //         .whileTrue(new IntakeCommand(m_shooterSubsystem));

        /*
         * =========================================
         * | OPERATOR CONTROLS |
         * =========================================
         */

        // Manual movement of the elevator
        // var operatorRightStickY = new Trigger(() ->
        //         Math.abs(operatorController.getRawAxis(Axis.kRightY)) >
        // ControllerConstants.kDeadzone);
        // operatorRightStickY.whileTrue(new ElevatorSpeedCommand(
        //         m_elevatorSubsystem, () -> -1.0 * operatorController.getRawAxis(Axis.kRightY)));

        var operatorLeftStickY = new Trigger(
                () -> Math.abs(operatorController.getRawAxis(Axis.kLeftY)) > ControllerConstants.kDeadzone);
        operatorLeftStickY.whileTrue(new AlgaePivotCommand(
                m_algaeSubsystem, () -> -1.0 * 0.5 * operatorController.getRawAxis(Axis.kLeftY)));

        // Manual elevator positions
        new JoystickButton(
                        operatorController,
                        ControllerConstants.Button.kLeftMenu) // This command zeros the elevator position
                .whileTrue(new ElevatorZeroPositionCommand(m_elevatorSubsystem));
        new JoystickButton(
                        operatorController,
                        ControllerConstants.Button
                                .kRightMenu) // This commmand goes to the zero elevator position
                .whileTrue(new ZeroCommand(m_elevatorSubsystem, m_shooterSubsystem));
        new JoystickButton(operatorController, ControllerConstants.Button.kA)
                .whileTrue(new ScoreL1Command(m_elevatorSubsystem, m_shooterSubsystem));
        new JoystickButton(operatorController, ControllerConstants.Button.kB)
                .whileTrue(new ScoreL2Command(m_elevatorSubsystem, m_shooterSubsystem));
        new JoystickButton(operatorController, ControllerConstants.Button.kX)
                .whileTrue(new ScoreL3Command(m_elevatorSubsystem, m_shooterSubsystem));
        new JoystickButton(operatorController, ControllerConstants.Button.kY)
                .whileTrue(new ScoreL4Command(m_elevatorSubsystem, m_shooterSubsystem));

        new JoystickButton(operatorController, ControllerConstants.Button.kLeftBumper)
                .whileTrue(new AlgaeIntakeCommand(m_algaeSubsystem, 0.65))
                .whileFalse(new AlgaeIntakeCommand(m_algaeSubsystem, 0));
        new JoystickButton(operatorController, ControllerConstants.Button.kRightBumper)
                .whileTrue(new AlgaeIntakeCommand(m_algaeSubsystem, -0.65))
                .whileFalse(new AlgaeIntakeCommand(m_algaeSubsystem, 0));

        new Trigger(() -> operatorController.getPOV() == DPad.kUp)
                .whileTrue(new BargeCommand(m_elevatorSubsystem, m_shooterSubsystem));

        // Set scoring target
        // new Trigger(() -> operatorController.getPOV() == DPad.kDown)
        //         .onTrue(new SetScoringTargetCommand(m_shooterSubsystem, ScoringTarget.L1));
        // new Trigger(() -> operatorController.getPOV() == DPad.kRight)
        //         .onTrue(new SetScoringTargetCommand(m_shooterSubsystem, ScoringTarget.L2));
        // new Trigger(() -> operatorController.getPOV() == DPad.kUp)
        //         .onTrue(new SetScoringTargetCommand(m_shooterSubsystem, ScoringTarget.L3));
        // new Trigger(() -> operatorController.getPOV() == DPad.kLeft)
        //         .onTrue(new SetScoringTargetCommand(m_shooterSubsystem, ScoringTarget.L4));
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
