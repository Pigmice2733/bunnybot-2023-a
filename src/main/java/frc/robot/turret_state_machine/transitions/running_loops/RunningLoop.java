package frc.robot.turret_state_machine.transitions.running_loops;

import frc.robot.turret_state_machine.Transition;

public abstract class RunningLoop<State extends Enum<?>, RobotData> extends Transition<State, RobotData> {

    public RunningLoop(State runningState) {
        super(runningState);
    }

    @Override
    public final boolean shouldExecute(RobotData robotData) {
        return true;
    }

    @Override
    public final State execute(RobotData robotData) {
        run(robotData);
        return to;
    }

    protected abstract void run(RobotData robotData);
}
