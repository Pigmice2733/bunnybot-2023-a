// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pigmice.frc.lib.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.swerve.commands.DriveWithJoysticks;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.commands.turret.RunTurretStateMachine;
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
    public final Turret turret = new Turret();
    private final Vision vision = new Vision();
    private final Grabber grabber = new Grabber();
    private final Hood hood = new Hood();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();

    private SwerveDrivetrain drivetrain = new SwerveDrivetrain(DrivetrainConfig.SWERVE_CONFIG);

    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);
    private final Controls controls = new Controls(driver, operator);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain.setDefaultCommand(new DriveWithJoysticks(drivetrain,
                controls::getDriveSpeedX,
                controls::getDriveSpeedY, controls::getTurnSpeed, () -> true));

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
        new JoystickButton(driver, Button.kX.value).onTrue(Commands.runOnce(() -> drivetrain.resetOdometry()));

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
