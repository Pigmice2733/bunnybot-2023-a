package frc.robot.turret_state_machine.transitions;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.Constants.TurretConfig;
import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;
import frc.robot.turret_state_machine.Transition;

public class ManualPressed extends Transition<TurretState, TurretData> {

    /** Triggers a transition when the manual button is pressed. */
    public ManualPressed() {
        super(TurretState.Manual);
    }

    @Override
    public boolean shouldExecute(TurretData turretData) {
        return Math.abs(turretData.manualRotationSpeed) >= Constants.AXIS_THRESHOLD;
    }

    @Override
    public TurretState execute(TurretData turretData) {
        turretData.setTurretConstraints
                .accept(new Constraints(TurretConfig.MAX_VELOCITY, TurretConfig.MAX_ACCELERATION));
        return to;
    }
}
