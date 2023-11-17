// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConfig;
import frc.robot.commands.actions.ShootTarget;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AutoShooter extends CommandBase {
    private final Hood hood;
    private final Indexer indexer;
    private final Shooter shooter;
    private final Turret turret;
    private final Vision vision;

    private boolean isShooting;
    private ShootTarget shootCommand;

    /** Automatically runs the shooter when there is a target in range. */
    public AutoShooter(Hood hood, Indexer indexer, Shooter shooter, Turret turret, Vision vision) {
        this.hood = hood;
        this.indexer = indexer;
        this.shooter = shooter;
        this.turret = turret;
        this.vision = vision;

        addRequirements(hood, shooter, indexer);
    }

    @Override
    public void execute() {
        if (vision.getCurrentTarget() != null
                && turret.getTurretVelocity() < TurretConfig.MAX_FIRE_VELOCITY) {
            if (!isShooting) {
                isShooting = true;
                indexer.spinBeltForward().schedule();
                shootCommand = new ShootTarget(hood, shooter, vision);
            }
        } else if (isShooting) {
            isShooting = false;
            indexer.stopBelt().schedule();
            shootCommand.cancel();
        }
    }
}
