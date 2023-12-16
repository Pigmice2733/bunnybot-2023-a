package frc.robot.commands.auto_routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.PauseTurret;
import frc.robot.commands.functions.RepeatFireShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoShooting extends SequentialCommandGroup {
    public AutoShooting(Turret turret, Shooter shooter, Indexer indexer) {
        addCommands(new PauseTurret(turret).withTimeout(5),
                new RepeatFireShooter(indexer, shooter));
    }
}
