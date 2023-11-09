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
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.GrabberConfig.ArmPosition;
import frc.robot.commands.RunTurretStateMachine;
import frc.robot.commands.functions.AutoShooter;
import frc.robot.commands.functions.EjectAll;
import frc.robot.commands.functions.IntakeAndShoot;
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
    private final Hood hood;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;
    public final Turret turret;
    private final Vision vision;

    private final XboxController driver;
    private final XboxController operator;
    private final Controls controls;

    private final IntakeAndShoot autoBallCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain = new SwerveDrivetrain(DrivetrainConfig.SWERVE_CONFIG);
        grabber = new Grabber();
        hood = new Hood();
        indexer = new Indexer();
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();
        vision = new Vision();

        driver = new XboxController(0);
        operator = new XboxController(1);
        controls = new Controls(driver, operator);

        autoBallCommand = new IntakeAndShoot(intake, indexer, shooter, turret);
        drivetrain.setDefaultCommand(new DriveWithJoysticksSwerve(drivetrain,
                controls::getDriveSpeedX,
                controls::getDriveSpeedY,
                controls::getTurnSpeed,
                () -> true));
        indexer.setDefaultCommand(autoBallCommand);
        intake.setDefaultCommand(autoBallCommand);
        shooter.setDefaultCommand(autoBallCommand);
        turret.setDefaultCommand(new RunTurretStateMachine(turret, vision,
                controls::getManualTurretRotationSpeed));

        configureButtonBindings();
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
        /*
         * Driver
         */

        // B (press) - Reset Odometry
        new JoystickButton(driver, Button.kB.value)
                .onTrue(new InstantCommand(drivetrain::resetOdometry));

        // TODO: add slow mode to swervedrivetrain in robolib
        // Y (hold) - drivetrain slow mode

        /*
         * Operator
         */

        // Right Bumper (toggle) - toggle auto shooter
        new JoystickButton(operator, Button.kRightBumper.value)
                .toggleOnTrue(new AutoShooter(indexer, shooter, turret));

        // X (hold) - fire shooter
        new JoystickButton(operator, Button.kX.value)
                .whileTrue(new RepeatFireShooter(indexer, shooter))
                .onFalse(shooter.stopFlywheel());

        // B (hold) - intake bunnies
        new JoystickButton(operator, Button.kX.value)
                .onTrue(Commands.parallel(grabber.setTargetArmAngleCommand(ArmPosition.DOWN),
                        grabber.runFlywheelsIntakeCommand()))
                .onFalse(Commands.parallel(grabber.setTargetArmAngleCommand(ArmPosition.UP),
                        grabber.stopFlywheelsCommand()));

        // Y (hold) - eject bunnies
        new JoystickButton(operator, Button.kX.value)
                .onTrue(Commands.parallel(grabber.setTargetArmAngleCommand(ArmPosition.MIDDLE),
                        grabber.runFlywheelsEjectCommand()))
                .onFalse(Commands.parallel(grabber.setTargetArmAngleCommand(ArmPosition.UP),
                        grabber.stopFlywheelsCommand()));

        // Left Bumper (hold) - eject balls through intake
        new JoystickButton(operator, Button.kLeftBumper.value)
                .whileTrue(new EjectAll(intake, indexer, shooter));
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
