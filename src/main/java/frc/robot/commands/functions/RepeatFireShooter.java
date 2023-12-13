// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functions;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ControllerRumbler;
import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.ShooterConfig;
import frc.robot.commands.actions.FeedShooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RepeatFireShooter extends SequentialCommandGroup {
    /** Fires the shooter at a set interval until canceled. */
    public RepeatFireShooter(Indexer indexer, Shooter shooter) {
        addCommands(
                shooter.setFlywheelSpeed(ShooterConfig.DEFAULT_OUTPUT),
                Commands.waitSeconds(AutoConfig.SHOOTER_SPINUP_TIME),
                Commands.repeatingSequence(
                        new FeedShooter(indexer),
                        Commands.runOnce(() -> ControllerRumbler.rumbleBoth(RumbleType.kBothRumble, 0.2, 1)),
                        Commands.waitSeconds(AutoConfig.TIME_BETWEEN_SHOTS)),
                shooter.stopFlywheel());
    }
}
