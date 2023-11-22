// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.actions.FeedShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;

public class AutoIndexer extends CommandBase {
    private final Indexer indexer;
    private final Turret turret;

    private boolean isMoving;
    private FeedShooter indexerCommand;

    /** Automatically runs the indexer when there is a target in range. */
    public AutoIndexer(Indexer indexer, Turret turret) {
        this.indexer = indexer;
        this.turret = turret;

        addRequirements(indexer);
    }

    @Override
    public void execute() {
        if (turret.hasTarget()) {
            if (!isMoving) {
                isMoving = true;
                indexerCommand = new FeedShooter(indexer);
            }
        } else if (isMoving) {
            isMoving = false;
            indexerCommand.cancel();
        }
    }
}
