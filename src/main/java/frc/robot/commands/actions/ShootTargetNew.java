package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConfig;
import frc.robot.Constants.VisionConfig;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShootTargetNew extends CommandBase {
    private final Hood hood;
    private final Shooter shooter;
    private final Vision vision;

    private double angle;

    /**
     * Sets the hood to the correct angle and the shooter flywheel to the correct
     * speed to shoot at the located target, using linear interpolation on a set of
     * test values.
     */
    public ShootTargetNew(Hood hood, Shooter shooter, Vision vision) {
        this.hood = hood;
        this.shooter = shooter;
        this.vision = vision;

        angle = 0;

        addRequirements(hood, shooter);
    }

    @Override
    public void execute() {
        angle = Math.atan((VisionConfig.TARGET_HEIGHT_METERS -
                VisionConfig.TURRET_HEIGHT_METERS) / vision.distanceToTarget());
        hood.setTargetRotation(90 - angle);
        shooter.outputToMotor(ShooterConfig.MAX_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
    }
}
