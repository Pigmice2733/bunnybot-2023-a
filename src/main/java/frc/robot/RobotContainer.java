// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.DriveWithJoysticksSwerve;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.GrabberConfig;
import frc.robot.Constants.GrabberConfig.ArmPosition;
import frc.robot.Constants.IntakeConfig;
import frc.robot.commands.RunTurretStateMachine;
import frc.robot.commands.actions.ManualHood;
import frc.robot.commands.actions.ZeroGrabber;
import frc.robot.commands.functions.AutoShooter;
import frc.robot.commands.functions.EjectAll;
import frc.robot.commands.functions.RepeatFireShooter;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final SwerveDrivetrain drivetrain;
    private final Grabber grabber;
    // private final Hood hood;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;
    // public final Turret turret;
    // private final Vision vision;

    private final XboxController driver;
    private final XboxController operator;
    private final Controls controls;

    // private final AutoShooter autoBallCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain = new SwerveDrivetrain(DrivetrainConfig.SWERVE_CONFIG);
        grabber = new Grabber();
        // hood = new Hood();
        indexer = new Indexer();
        intake = new Intake();
        shooter = new Shooter();
        // turret = new Turret();
        // vision = new Vision();

        driver = new XboxController(0);
        operator = new XboxController(1);
        controls = new Controls(driver, operator);

        // autoBallCommand = new AutoShooter(hood, indexer, shooter, turret, vision);
        drivetrain.setDefaultCommand(new DriveWithJoysticksSwerve(drivetrain,
                controls::getDriveSpeedX,
                controls::getDriveSpeedY,
                controls::getTurnSpeed,
                () -> true));
        // hood.setDefaultCommand(new ManualHood(hood, controls::getManualHoodSpeed));
        // indexer.setDefaultCommand(autoBallCommand);
        // shooter.setDefaultCommand(autoBallCommand);
        // turret.setDefaultCommand(new ManuelTurret(turret,
        // controls::getManualTurretRotationSpeed))
        // turret.setDefaultCommand(new RunTurretStateMachine(turret, vision,
        // controls::getManualTurretRotationSpeed));
        // intake.spinForward().schedule();

        configureButtonBindings();
    }

    public void onEnable() {
        intake.spinForward().schedule();
        indexer.spinFeederBackwards().schedule();
        grabber.resetPID();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* DRIVER */

        // B (press) - Reset Odometry
        new JoystickButton(driver, Button.kX.value)
                .onTrue(new InstantCommand(drivetrain::resetOdometry));

        /* OPERATOR */

        // Grab bunny from floor
        new POVButton(operator, 180) // down
                .onTrue(Commands.parallel(
                        grabber.setTargetArmAngleCommand(ArmPosition.GROUND),
                        grabber.runFlywheelsIntakeCommand()))
                .onFalse(Commands.parallel(
                        grabber.setTargetArmAngleCommand(ArmPosition.STORE),
                        grabber.stopFlywheelsCommand()));

        // Eject bunny
        new POVButton(operator, 270) // left
                .onTrue(Commands.sequence(
                        grabber.setTargetArmAngleCommand(ArmPosition.TOTE),
                        intake.stopWheels(),
                        Commands.waitSeconds(0.5),
                        grabber.runFlywheelsEjectCommand()))
                .onFalse(Commands.parallel(
                        intake.spinForward(),
                        grabber.stopFlywheelsCommand(),
                        grabber.setTargetArmAngleCommand(ArmPosition.STORE)));

        // Eject bunny
        new POVButton(operator, 90) // right
                .onTrue(Commands.sequence(
                        grabber.setTargetArmAngleCommand(ArmPosition.TOTE),
                        intake.stopWheels(),
                        Commands.waitSeconds(0.5),
                        grabber.runFlywheelsEjectCommand()))
                .onFalse(Commands.parallel(
                        intake.spinForward(),
                        grabber.stopFlywheelsCommand(),
                        grabber.setTargetArmAngleCommand(ArmPosition.STORE)));

        // Place bunny on ground
        new POVButton(operator, 0) // up
                .whileTrue(Commands.sequence(
                        grabber.setTargetArmAngleCommand(ArmPosition.START),
                        intake.stopWheels(),
                        Commands.waitSeconds(0.5),
                        grabber.runFlywheelsEjectCommand()))
                .onFalse(Commands.parallel(
                        intake.spinForward(),
                        grabber.stopFlywheelsCommand()));

        // Right Bumper (toggle) - toggle auto shooter
        // new JoystickButton(operator, Button.kRightBumper.value)
        // .toggleOnTrue(new AutoShooter(hood, indexer, shooter, turret,
        // vision));

        // Left Bumper (hold) - eject balls through intake
        // new JoystickButton(operator, Button.kLeftBumper.value)
        // .whileTrue(new EjectAll(intake, indexer, shooter));

        // X (hold) - fire shooter
        new JoystickButton(operator, Button.kX.value)
                .whileTrue(new RepeatFireShooter(indexer, shooter))
                .onFalse(Commands.parallel(shooter.stopFlywheel(), indexer.spinFeederBackwards()));

        // Throw bunny
        new JoystickButton(operator, Button.kStart.value)
                .whileTrue(Commands.sequence(grabber.setTargetArmAngleCommand(ArmPosition.THROW),
                        /*
                         * grabber.setControllerConstraints(GrabberConfig.MAX_VELOCITY * 3,
                         * GrabberConfig.MAX_ACCELERATION * 3,
                         * GrabberConfig.ARM_P * 3),
                         */
                        Commands.waitSeconds(0.3),
                        Commands.runOnce(() -> grabber.outputToFlywheelsMotor(1)),
                        Commands.waitSeconds(0.3),
                        grabber.stopFlywheelsCommand()));

        // Throw bunny old
        // new JoystickButton(operator, Button.kStart.value)
        // .whileTrue(Commands.sequence(grabber.setTargetArmAngleCommand(ArmPosition.START),
        // /*
        // * grabber.setControllerConstraints(GrabberConfig.MAX_VELOCITY * 3,
        // * GrabberConfig.MAX_ACCELERATION * 3,
        // * GrabberConfig.ARM_P * 3),
        // */
        // Commands.waitSeconds(0.3),
        // Commands.runOnce(() -> grabber.outputToFlywheelsMotor(1)),
        // Commands.waitSeconds(0.3),
        // grabber.stopFlywheelsCommand()))
        // /*
        // * .onFalse(grabber.setControllerConstraints(GrabberConfig.MAX_VELOCITY,
        // * GrabberConfig.MAX_ACCELERATION, GrabberConfig.ARM_P))
        // */;

        new JoystickButton(operator, Button.kA.value)
                .whileTrue(new ZeroGrabber(grabber));

        // // Y (hold) - eject bunnies
        // new JoystickButton(operator, Button.kY.value)
        // .onTrue(Commands.parallel(
        // grabber.setTargetArmAngleCommand(ArmPosition.MIDDLE),
        // grabber.runFlywheelsEjectCommand()))
        // .onFalse(Commands.parallel(
        // grabber.setTargetArmAngleCommand(ArmPosition.UP),
        // grabber.stopFlywheelsCommand()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
