// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConfig;
import frc.robot.commands.actions.FeedShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AutoIndexer extends CommandBase {
    private final Indexer indexer;
    private final Turret turret;
    private final Vision vision;

    private boolean isMoving;
    private FeedShooter indexerCommand;

    /** Automatically runs the indexer when there is a target in range. */
    public AutoIndexer(Indexer indexer, Turret turret, Vision vision) {
        this.indexer = indexer;
        this.turret = turret;
        this.vision = vision;

        addRequirements(indexer);
    }

    @Override
    public void execute() {
        if (vision.getCurrentTarget() != null
                && turret.getTurretVelocity() < TurretConfig.MAX_FIRE_VELOCITY) {
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
