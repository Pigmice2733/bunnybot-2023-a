package frc.robot.commands.actions;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class ManualHood extends CommandBase {
    private final Hood hood;
    private DoubleSupplier rotationSpeed;

    public ManualHood(Hood hood, DoubleSupplier speed) {
        this.hood = hood;
        rotationSpeed = speed;

        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.outputToMotor(rotationSpeed.getAsDouble());
    }
}
