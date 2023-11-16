package frc.robot.turret_state_machine.transitions;

import frc.robot.Constants.TurretConfig;
import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;
import frc.robot.turret_state_machine.Transition;

public class RightBoundReached extends Transition<TurretState, TurretData> {

    /** Triggers a transition when the turret has reached its right boundary. */
    public RightBoundReached() {
        super(TurretState.WanderLeft);
    }

    @Override
    public boolean shouldExecute(TurretData turretData) {
        return turretData.turretRotation > TurretConfig.WANDER_LIMIT;
    }

    @Override
    public TurretState execute(TurretData turretData) {
        return to;
    }
}
