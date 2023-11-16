package frc.robot.turret_state_machine.transitions;

import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;
import frc.robot.turret_state_machine.Transition;

public class TargetFound extends Transition<TurretState, TurretData> {

    /** Triggers a transition when a valid target can be seen. */
    public TargetFound() {
        super(TurretState.TrackTarget);
    }

    @Override
    public boolean shouldExecute(TurretData turretData) {
        return turretData.hasTarget;
    }

    @Override
    public TurretState execute(TurretData turretData) {
        return to;
    }
}
