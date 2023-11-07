package frc.robot.turret_state_machine.transitions.running_loops;

import frc.robot.turret_state_machine.Transition;

public abstract class RunningLoop<State extends Enum<?>, RobotData> extends Transition<State, RobotData> {

    public RunningLoop(State runningState) {
        super(runningState, runningState);
    }

    @Override
    public final boolean shouldExecute(Object robotData) {
        return true;
    }

    @Override
    public final State execute(Object robotData) {
        run();
        return to;
    }

    protected abstract void run();
}
