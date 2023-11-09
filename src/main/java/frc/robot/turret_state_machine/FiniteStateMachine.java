package frc.robot.turret_state_machine;

import java.util.HashMap;
import java.util.Map;

public class FiniteStateMachine<State extends Enum<?>, RobotData> {
    private State currentState;
    private final Map<State, Transition<State, RobotData>[]> nodeTransitionMap;

    public FiniteStateMachine(State initialState, Map<State, Transition<State, RobotData>[]> nodeTransitionMap) {
        currentState = initialState;
        this.nodeTransitionMap = nodeTransitionMap;
    }

    public FiniteStateMachine(State initialState) {
        currentState = initialState;
        nodeTransitionMap = new HashMap<State, Transition<State, RobotData>[]>();
    }

    public boolean execute(RobotData robotData) {
        Transition<State, RobotData>[] transitions = nodeTransitionMap.get(currentState);

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

    @SafeVarargs
    public final void addTransitionsFromState(State state, Transition<State, RobotData>... transitions) {
        nodeTransitionMap.put(state, transitions);
    }
}
