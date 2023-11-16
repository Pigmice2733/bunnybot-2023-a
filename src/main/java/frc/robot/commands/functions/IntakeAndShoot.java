package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class IntakeAndShoot extends ParallelCommandGroup {
    /** Runs the intake and shoots when there is a target in range. */
    public IntakeAndShoot(Intake intake, Indexer indexer, Shooter shooter, Turret turret, Vision vision) {
        addCommands(intake.spinForward(), new AutoShooter(indexer, shooter, turret, vision));
        addRequirements(intake, indexer, shooter);
    }
}
