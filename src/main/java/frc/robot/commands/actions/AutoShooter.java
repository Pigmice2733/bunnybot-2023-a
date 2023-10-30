// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoShooter extends CommandBase {
    private final Turret turret;

    private boolean isShooting;
    private RunShooter shootCommand;

    /** Automatically runs the shooter, but only when there is a target in range */
    public AutoShooter(Shooter shooter, Turret turret, Indexer indexer) {
        this.turret = turret;

        shootCommand = new RunShooter(indexer, shooter);
        addRequirements(shooter, indexer);
    }

    @Override
    public void execute() {
        if (turret.hasTarget()) {
            if (!isShooting) {
                isShooting = true;
                shootCommand.schedule();
            }
        } else if (isShooting) {
            isShooting = false;
            shootCommand.cancel();
        }
    }
}
