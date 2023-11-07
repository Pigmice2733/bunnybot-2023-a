package frc.robot.turret_state_machine;

import java.util.List;
import java.util.Map;

public class FiniteStateMachine<State extends Enum<?>, RobotData> {
    private State currentState;
    private final Map<State, List<Transition<State, RobotData>>> nodeTransitionMap;

    public FiniteStateMachine(State initialState, Map<State, List<Transition<State, RobotData>>> nodeTransitionMap) {
        currentState = initialState;
        this.nodeTransitionMap = nodeTransitionMap;
    }

    public boolean execute(RobotData robotData) {
        List<Transition<State, RobotData>> transitions = nodeTransitionMap.get(currentState);

        if (transitions == null)
            return false;

        for (Transition<State, RobotData> t : transitions) {
            if (t.shouldExecute(robotData)) {
                currentState = t.execute(robotData);
                return true;
            }
        }

        return false;
    }
}
