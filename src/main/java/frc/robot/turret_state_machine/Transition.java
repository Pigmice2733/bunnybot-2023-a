package frc.robot.turret_state_machine;

public abstract class Transition<State extends Enum<?>, RobotData> {
    protected final State from;
    protected final State to;

    public Transition(State from, State to) {
        this.from = from;
        this.to = to;
    }

    public abstract boolean shouldExecute(RobotData robotData);

    public abstract State execute(RobotData robotData);
}
