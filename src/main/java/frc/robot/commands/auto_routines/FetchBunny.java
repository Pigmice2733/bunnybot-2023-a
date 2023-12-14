// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_routines;

import java.util.HashMap;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.path_following.FollowPathSwerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.GrabberConfig.ArmPosition;
import frc.robot.subsystems.Grabber;

public class FetchBunny extends SequentialCommandGroup {
  /** Creates a new FetchBunny. */
  public FetchBunny(SwerveDrivetrain drivetrain, Grabber grabber) {
    HashMap<String, Command> pathEvents = new HashMap<String, Command>();

    pathEvents.put("grabber-up",
        new ParallelCommandGroup(grabber.setTargetArmAngleCommand(ArmPosition.TOTE), grabber.stopFlywheelsCommand()));
    pathEvents.put("grabber-down", new SequentialCommandGroup(grabber.setTargetArmAngleCommand(ArmPosition.STOW)));
    pathEvents.put("drop-bunny", grabber.runFlywheelsEjectCommand());

    addCommands(
        new ParallelCommandGroup(new FollowPathSwerve(drivetrain, AutoConfig.FETCH_BUNNY_PATH, pathEvents, false)));
  }
}
