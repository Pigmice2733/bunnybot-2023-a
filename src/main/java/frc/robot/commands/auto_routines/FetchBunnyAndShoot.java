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
import frc.robot.commands.functions.RepeatFireShooter;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class FetchBunnyAndShoot extends SequentialCommandGroup {
        /** Creates a new FetchBunny. */
        public FetchBunnyAndShoot(SwerveDrivetrain drivetrain, Grabber grabber, Turret turret, Indexer indexer,
                        Shooter shooter) {
                HashMap<String, Command> pathEvents = new HashMap<String, Command>();

                pathEvents.put("lower-grabber",
                                new ParallelCommandGroup(Commands.runOnce(() -> turret.resetPID()),
                                                Commands.runOnce(() -> turret.startPID()),
                                                grabber.setTargetArmAngleCommand(ArmPosition.TOTE),
                                                grabber.runFlywheelsIntakeCommand(),
                                                new RepeatFireShooter(indexer, shooter).withTimeout(7).andThen(
                                                                Commands.parallel(shooter.idleFlywheel(),
                                                                                indexer.spinFeederBackwards()),
                                                                Commands.runOnce(() -> turret.startPID()))));
                pathEvents.put("raise-grabber",
                                new SequentialCommandGroup(grabber.setTargetArmAngleCommand(ArmPosition.STOW),
                                                grabber.stopFlywheelsCommand()));

                pathEvents.put("drop-bunny", Commands.runOnce(() -> Commands.sequence(
                                grabber.setTargetArmAngleCommand(ArmPosition.TOTE),
                                Commands.waitSeconds(0.5),
                                grabber.runFlywheelsEjectCommand(),
                                Commands.waitSeconds(2),
                                grabber.stopFlywheelsCommand(),
                                grabber.setTargetArmAngleCommand(ArmPosition.STORE)).schedule()));

                addCommands(
                                new ParallelCommandGroup(Commands.runOnce(() -> turret.stopPID()),
                                                new FollowPathSwerve(drivetrain, AutoConfig.FETCH_BUNNY_PATH,
                                                                pathEvents, false)),
                                Commands.sequence(
                                                grabber.setTargetArmAngleCommand(ArmPosition.TOTE),
                                                Commands.waitSeconds(0.5),
                                                grabber.runFlywheelsEjectCommand(),
                                                Commands.waitSeconds(2),
                                                grabber.stopFlywheelsCommand(),
                                                grabber.setTargetArmAngleCommand(ArmPosition.STORE)));
        }
}
