package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class IntakeAndShoot extends ParallelCommandGroup {
    /** Runs the intake and shoots when there is a target in range. */
    public IntakeAndShoot(Intake intake, Indexer indexer, Shooter shooter, Turret turret) {
        addCommands(intake.spinForward(), new AutoShooter(indexer, shooter, turret));
    }
}
