package frc.robot.turret_state_machine.transitions;

import frc.robot.Constants;
import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;
import frc.robot.turret_state_machine.Transition;

public class ManualReleased extends Transition<TurretState, TurretData> {

    /** Triggers a transition when the manual button is not pressed. */
    public ManualReleased() {
        super(TurretState.BeginWander);
    }

    @Override
    public boolean shouldExecute(TurretData turretData) {
        return Math.abs(turretData.manualRotationSpeed) < Constants.AXIS_THRESHOLD;
    }

    @Override
    public TurretState execute(TurretData turretData) {
        return to;
    }
}
