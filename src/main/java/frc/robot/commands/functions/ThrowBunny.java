// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functions;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ControllerRumbler;
import frc.robot.Constants.GrabberConfig.ArmPosition;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;

public class ThrowBunny extends SequentialCommandGroup {
  public ThrowBunny(SwerveDrivetrain drivetrain, Grabber grabber, Intake intake) {
    addRequirements(drivetrain, grabber);

    // Rotation2d drivetrainRotation = drivetrain.getHeading();
    // System.out.println("Rotation " + drivetrainRotation.getDegrees());

    // Translation2d directionVector = new Translation2d(1,
    // 0).rotateBy(drivetrainRotation);
    // System.out.println("Direction " + directionVector);

    addCommands(grabber.setTargetArmAngleCommand(ArmPosition.STOW),
        /*
         * grabber.setControllerConstraints(GrabberConfig.MAX_VELOCITY * 3,
         * GrabberConfig.MAX_ACCELERATION * 3,
         * GrabberConfig.ARM_P * 3),
         */
        intake.stopWheels(),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> drivetrain.driveChassisSpeeds(new ChassisSpeeds(-5, 0, 0))),
        Commands.waitSeconds(0.3),
        grabber.setTargetArmAngleCommand(ArmPosition.GROUND),
        Commands.waitSeconds(0.55),
        Commands.runOnce(() -> grabber.outputToFlywheelsMotor(-1)),
        Commands.waitSeconds(0.1),
        Commands.runOnce(() -> drivetrain.driveChassisSpeeds(new ChassisSpeeds(0, 0, 0))),
        Commands.runOnce(() -> ControllerRumbler.rumblerOperator(RumbleType.kBothRumble, 1, 1)),
        Commands.waitSeconds(0.3),
        grabber.stopFlywheelsCommand(),
        intake.spinForward());
  }
}
