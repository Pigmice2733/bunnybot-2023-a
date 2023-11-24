// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.actions.RunAndFeed;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoShooter extends CommandBase {
    private final Indexer indexer;
    private final Shooter shooter;
    private final Turret turret;

    private boolean isShooting;
    private RunAndFeed shootCommand;

    /** Automatically runs the shooter when there is a target in range. */
    public AutoShooter(Indexer indexer, Shooter shooter, Turret turret) {
        this.indexer = indexer;
        this.shooter = shooter;
        this.turret = turret;

        addRequirements(shooter, indexer);
    }

    @Override
    public void execute() {
        if (turret.hasTarget()) {
            if (!isShooting) {
                isShooting = true;
                shootCommand = new RunAndFeed(indexer, shooter);
            }
        } else if (isShooting) {
            isShooting = false;
            shootCommand.cancel();
        }
    }
}
