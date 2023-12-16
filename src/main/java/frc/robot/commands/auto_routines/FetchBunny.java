// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_routines;

import java.util.HashMap;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.path_following.FollowPathSwerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.GrabberConfig.ArmPosition;
import frc.robot.subsystems.Grabber;

public class FetchBunny extends SequentialCommandGroup {
    /** Creates a new FetchBunny. */
    public FetchBunny(SwerveDrivetrain drivetrain, Grabber grabber) {
        HashMap<String, Command> pathEvents = new HashMap<String, Command>();

        pathEvents.put("lower-grabber",
                new ParallelCommandGroup(grabber.setTargetArmAngleCommand(ArmPosition.TOTE),
                        grabber.runFlywheelsIntakeCommand()));
        pathEvents.put("raise-grabber", new SequentialCommandGroup(grabber.setTargetArmAngleCommand(ArmPosition.STOW),
                grabber.stopFlywheelsCommand()));

        pathEvents.put("drop-bunny", Commands.sequence(
                grabber.setTargetArmAngleCommand(ArmPosition.TOTE),
                Commands.waitSeconds(0.5),
                grabber.runFlywheelsEjectCommand(),
                Commands.waitSeconds(2),
                grabber.stopFlywheelsCommand(),
                grabber.setTargetArmAngleCommand(ArmPosition.STORE)));

        addCommands(
                new ParallelCommandGroup(
                        new FollowPathSwerve(drivetrain, AutoConfig.FETCH_BUNNY_PATH_NO_SHOOT, pathEvents, false)),
                Commands.sequence(
                        grabber.setTargetArmAngleCommand(ArmPosition.TOTE),
                        Commands.waitSeconds(0.5),
                        grabber.runFlywheelsEjectCommand(),
                        Commands.waitSeconds(2),
                        grabber.stopFlywheelsCommand(),
                        grabber.setTargetArmAngleCommand(ArmPosition.STORE)));
    }
}
