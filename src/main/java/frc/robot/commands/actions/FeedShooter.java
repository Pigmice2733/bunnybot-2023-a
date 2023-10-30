// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.subsystems.Indexer;

public class FeedShooter extends SequentialCommandGroup {
  /** Feeds a single ball into the shooter */
  public FeedShooter(Indexer indexer) {
    addCommands(
        indexer.spinBeltForward(),
        Commands.waitSeconds(AutoConfig.FEED_SHOOTER_INDEX_TIME),
        indexer.stopBelt());
  }
}
