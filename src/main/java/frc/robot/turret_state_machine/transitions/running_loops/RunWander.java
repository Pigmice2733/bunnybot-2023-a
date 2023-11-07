package frc.robot.turret_state_machine.transitions.running_loops;

import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;

public class RunWander extends RunningLoop<TurretState, TurretData> {

    public RunWander(TurretState runningState, WanderDirection direction) {
        super(runningState);
    }

    @Override
    protected void run() {
        throw new UnsupportedOperationException("Unimplemented method 'run'");
    }

    public enum WanderDirection {
        Left,
        Right
    }
}
