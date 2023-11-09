package frc.robot.turret_state_machine;

public abstract class Transition<State extends Enum<?>, RobotData> {
    protected final State to;

    public Transition(State to) {
        this.to = to;
    }

    public abstract boolean shouldExecute(RobotData robotData);

    public abstract State execute(RobotData robotData);
}
