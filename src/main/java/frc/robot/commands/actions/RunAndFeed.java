// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class RunAndFeed extends SequentialCommandGroup {
    /** Runs the shooter while feeding it. */
    public RunAndFeed(Indexer indexer, Shooter shooter) {
        addCommands(
                shooter.setFlywheelSpeed(ShooterConfig.DEFAULT_OUTPUT),
                new WaitCommand(AutoConfig.SHOOTER_SPINUP_TIME),
                new FeedShooter(indexer),
                shooter.idleFlywheel());
    }
}
