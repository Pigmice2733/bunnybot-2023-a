package frc.robot.commands.actions;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShootTarget extends CommandBase {
    private final Hood hood;
    private final Shooter shooter;
    private final Vision vision;

    private final Double[] inputs;
    private final Double[][] outputs;

    private double distance, speed, angle;
    private int index;

    /**
     * Sets the hood to the correct angle and the shooter flywheel to the correct speed to shoot at
     * the located target, using linear interpolation on a set of test values.
     */
    public ShootTarget(Hood hood, Shooter shooter, Vision vision) {
        this.hood = hood;
        this.shooter = shooter;
        this.vision = vision;

        inputs = ShooterConfig.TEST_INPUTS;
        outputs = ShooterConfig.TEST_OUTPUTS;

        distance = speed = angle = 0;

        addRequirements(hood, shooter);
    }

    @Override
    public void execute() {
        distance = vision.distanceToTarget();

        index = Arrays.binarySearch(inputs, distance) + 1;
        // binarySearch returns index if found or (-ceiling - 1) if not found
        if (index >= 0) {
            angle = outputs[index - 1][0];
            speed = outputs[index - 1][1];
        } else {
            // linear interpolation
            angle = ((outputs[-index][0] - outputs[-index - 1][0]) * (distance - inputs[-index - 1])
                    / (inputs[-index] - inputs[-index - 1])) + outputs[-index - 1][0];
            speed = ((outputs[-index][1] - outputs[-index - 1][1]) * (distance - inputs[-index - 1])
                    / (inputs[-index] - inputs[-index - 1])) + outputs[-index - 1][1];
        }
        hood.setTargetRotation(angle);
        shooter.outputToMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
    }
}
