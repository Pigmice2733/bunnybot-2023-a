package frc.robot.turret_state_machine.transitions;

import frc.robot.Constants.TurretConfig;
import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;
import frc.robot.turret_state_machine.Transition;

public class LeftBoundReached extends Transition<TurretState, TurretData> {

    public LeftBoundReached(TurretState to) {
        super(to);
    }

    @Override
    public boolean shouldExecute(TurretData turretData) {
        return turretData.turretRotation < -TurretConfig.WANDER_LIMIT;
    }

    @Override
    public TurretState execute(TurretData turretData) {
        return to;
    }
}
