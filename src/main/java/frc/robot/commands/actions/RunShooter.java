// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class RunShooter extends SequentialCommandGroup {
    /** Runs and feeds the shooter */
    public RunShooter(Indexer indexer, Shooter shooter) {
        addCommands(
                shooter.setFlywheelSpeed(ShooterConfig.DEFAULT_OUTPUT),
                Commands.waitSeconds(AutoConfig.SHOOTER_SPINUP_TIME),
                new FeedShooter(indexer));
    }
}
