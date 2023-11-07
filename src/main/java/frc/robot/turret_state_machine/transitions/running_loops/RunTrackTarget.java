package frc.robot.turret_state_machine.transitions.running_loops;

import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;

public class RunTrackTarget extends RunningLoop<TurretState, TurretData> {

    public RunTrackTarget(TurretState runningState) {
        super(runningState);
    }

    @Override
    protected void run() {
        throw new UnsupportedOperationException("Unimplemented method 'run'");
    }
}
