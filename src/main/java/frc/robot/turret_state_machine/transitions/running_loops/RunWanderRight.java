package frc.robot.turret_state_machine.transitions.running_loops;

import frc.robot.Constants.TurretConfig;
import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;

public class RunWanderRight extends RunningLoop<TurretState, TurretData> {
    public RunWanderRight(TurretState runningState) {
        super(runningState);
    }

    @Override
    protected void run(TurretData turretData) {
        turretData.setTargetRotation.accept(TurretConfig.WANDER_LIMIT + 5);
    }
}
