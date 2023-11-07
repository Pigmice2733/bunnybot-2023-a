// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pigmice.frc.lib.drivetrain.swerve.commands.DriveWithJoysticksSwerve;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.ShooterConfig;
import frc.robot.Constants.GrabberConfig.ArmPosition;
import frc.robot.commands.RunTurretStateMachine;
import frc.robot.commands.functions.AutoShooter;
import frc.robot.commands.functions.EjectAll;
import frc.robot.commands.functions.IntakeAndShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;
// import frc.robot.subsystems.Hood;
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
    public final Drivetrain drivetrain;
    private final Grabber grabber;
    // private final Hood hood;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;
    public final Turret turret;
    private final Vision vision;

    private final XboxController driver;
    private final XboxController operator;
    private final Controls controls;

    private final IntakeAndShoot autoBallCommand;

    private boolean intakeToggle, forceShootToggle;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain = new Drivetrain(DrivetrainConfig.SWERVE_CONFIG);
        grabber = new Grabber();
        // hood = new Hood();
        indexer = new Indexer();
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();
        vision = new Vision();

        driver = new XboxController(0);
        operator = new XboxController(1);
        controls = new Controls(driver, operator);

        intakeToggle = forceShootToggle = false;

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

        new JoystickButton(driver, Button.kB.value)
                .onTrue(new InstantCommand(drivetrain::resetOdometry));
        new JoystickButton(driver, Button.kY.value)
                .onTrue(new InstantCommand(drivetrain::enableSlow))
                .onFalse(new InstantCommand(drivetrain::disableSlow));

        new POVButton(operator, 0)
                .onTrue(grabber.setTargetArmAngleCommand(ArmPosition.UP));
        new POVButton(operator, 90)
                .onTrue(grabber.setTargetArmAngleCommand(ArmPosition.MIDDLE));
        new POVButton(operator, 180)
                .onTrue(grabber.setTargetArmAngleCommand(ArmPosition.DOWN));
        new JoystickButton(operator, Button.kLeftBumper.value)
                .onTrue(new EjectAll(intake, indexer, shooter));
        new JoystickButton(operator, Button.kRightBumper.value)
                .toggleOnTrue(new AutoShooter(indexer, shooter, turret));
        new JoystickButton(operator, Button.kB.value)
                .onTrue(intakeToggle ? intake.spinForward() : intake.stopWheels());
        new JoystickButton(operator, Button.kY.value)
                .onTrue(forceShootToggle
                        ? shooter.setFlywheelSpeed(ShooterConfig.DEFAULT_OUTPUT)
                        : shooter.stopFlywheel());
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
