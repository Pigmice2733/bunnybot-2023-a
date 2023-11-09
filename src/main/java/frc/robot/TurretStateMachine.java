package frc.robot;

import java.util.Hashtable;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TurretConfig;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretStateMachine {

    private Hashtable<TurretStates, TurretState> states = new Hashtable<TurretStates, TurretState>();
    private TurretState currentState;

    private final Vision vision;
    private final Turret turret;

    private final DoubleSupplier manualRotationSpeed;

    /** Creates a state machine to control the turret's behavior. */
    public TurretStateMachine(Turret turret, Vision vision, DoubleSupplier manualRotationSpeed) {
        states.put(TurretStates.IDLE, new Idle());
        states.put(TurretStates.TRACK_TARGET, new TrackTarget());
        states.put(TurretStates.WANDER_LEFT, new WanderLeft());
        states.put(TurretStates.WANDER_RIGHT, new WanderRight());
        states.put(TurretStates.MANUAL, new ManualControl());

        currentState = states.get(TurretStates.IDLE);

        this.turret = turret;
        this.vision = vision;
        this.manualRotationSpeed = manualRotationSpeed;

        ShuffleboardHelper.addOutput("State", Constants.TURRET_TAB, () -> currentState.getClass().getName());

        ShuffleboardHelper.addOutput("Has Target", Constants.TURRET_TAB,
                () -> ((TrackTarget) getState(TurretStates.TRACK_TARGET)).targetInRange);
    }

    /** Switches the current state, and calls appropriate entry and exit methods. */
    public void setState(TurretStates stateType) {
        turret.resetRotationController();
        SmartDashboard.putString("State", stateType.name());
        TurretState newState = states.get(stateType);

        currentState.onStateExit();

        if (newState == null)
            setState(TurretStates.IDLE);
        else
            currentState = newState;

        currentState.onStateEntry();
    }

    /** @return the state the turret is currently in */
    public TurretState getCurrentState() {
        return currentState;
    }

    /** @return the instance of a state */
    public TurretState getState(TurretStates state) {
        return states.get(state);
    }

    /**
     * Periodically updates the state machine, including the {@code execute} method
     * of the current state.
     */
    public void updateStateMachine() {
        if (currentState == null)
            return;

        // If the manual control trigger is pressed, switch to the manual control state
        if (Math.abs(manualRotationSpeed.getAsDouble()) > Constants.AXIS_THRESHOLD) {
            setState(TurretStates.MANUAL);
        }

        currentState.execute();
    }

    // Implement this for any behavior of the turret
    public interface TurretState {
        /** Called once when the turret enters this state. */
        public default void onStateEntry() {
        };

        /** Called repeatedly while the turret is in this state. */
        public default void execute() {
        };

        /** Called once when the turret exits this state. */
        public default void onStateExit() {
        };
    }

    /** When the turret is idle, it will not move at all. */
    public class Idle implements TurretState {
    }

    /** Tracks a target until it is not seen, then switches to wander. */
    public class TrackTarget implements TurretState {
        private PhotonTrackedTarget target = null;
        public boolean targetInRange;

        @Override
        public void execute() {
            target = vision.getCurrentTarget();

            if (target == null) {
                // Wander in the direction the turret is currently rotating
                // TODO: test if it goes the right direction when it switches to wander
                if (Math.signum(turret.getTurretVelocity()) >= 0)
                    setState(TurretStates.WANDER_RIGHT);
                else
                    setState(TurretStates.WANDER_LEFT);
                return;
            }

            // Sets the target rotation to the current rotation plus the target's yaw
            turret.setTargetRotation(turret.getCurrentRotation() + target.getYaw());
            targetInRange = Math.abs(target.getYaw()) < TurretConfig.TARGET_YAW_TOLERANCE;
        }

        @Override
        public void onStateExit() {
            targetInRange = false;
        }
    }

    /** Rotates right until a target is found, or the wander boundary is reached. */
    public class WanderRight implements TurretState {
        double directionMultiplier, currentRotation;
        PhotonTrackedTarget target;
        double wanderLimit = TurretConfig.WANDER_LIMIT;

        @Override
        public void onStateEntry() {
            turret.setPIDConstraints(
                    new Constraints(TurretConfig.WANDER_VELOCITY, TurretConfig.WANDER_ACCELERATION));

            turret.setTargetRotation(wanderLimit + 5);
        }

        @Override
        public void execute() {
            target = vision.getCurrentTarget();
            currentRotation = turret.getCurrentRotation();

            // If a target is found, track it
            if (target != null) {
                setState(TurretStates.TRACK_TARGET);
                return;
            }

            // If the wander limit is reached, switch directions
            if (currentRotation > wanderLimit) {
                setState(TurretStates.WANDER_LEFT);
            }
        }

        @Override
        public void onStateExit() {
            turret.setPIDConstraints(
                    new Constraints(TurretConfig.MAX_VELOCITY, TurretConfig.MAX_ACCELERATION));
        }
    }

    /** Rotates left until a target is found, or the wander boundary is reached. */
    public class WanderLeft implements TurretState {
        double directionMultiplier, currentRotation;
        PhotonTrackedTarget target;
        double wanderLimit = TurretConfig.WANDER_LIMIT;

        @Override
        public void onStateEntry() {
            turret.setPIDConstraints(
                    new Constraints(TurretConfig.WANDER_VELOCITY, TurretConfig.WANDER_ACCELERATION));

            turret.setTargetRotation(-wanderLimit - 5);
        }

        @Override
        public void execute() {
            target = vision.getCurrentTarget();
            currentRotation = turret.getCurrentRotation();

            // If a target is found, track it
            if (target != null) {
                setState(TurretStates.TRACK_TARGET);
                return;
            }

            // If the wander limit is reached, switch directions
            if (currentRotation < -wanderLimit) {
                setState(TurretStates.WANDER_RIGHT);
            }
        }

        @Override
        public void onStateExit() {
            turret.setPIDConstraints(
                    new Constraints(TurretConfig.MAX_VELOCITY, TurretConfig.MAX_ACCELERATION));
        }
    }

    /** Manually controls the turret until the input is zero, then wander. */
    public class ManualControl implements TurretState {
        double manualSpeed;

        @Override
        public void execute() {
            manualSpeed = manualRotationSpeed.getAsDouble();

            // Switch back to wander state if the trigger is no longer pressed
            if (Math.abs(manualSpeed) < Constants.AXIS_THRESHOLD) {
                setState(TurretStates.WANDER_RIGHT);
                return;
            }

            turret.changeTargetRotation(manualSpeed);
        }
    }

    public static enum TurretStates {
        IDLE, TRACK_TARGET, WANDER_LEFT, WANDER_RIGHT, MANUAL
    }
}
