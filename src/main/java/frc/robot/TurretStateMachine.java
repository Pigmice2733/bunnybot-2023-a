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
        states.put(TurretStates.idle, new Idle());
        states.put(TurretStates.trackTarget, new TrackTarget());
        states.put(TurretStates.wanderLeft, new WanderLeft());
        states.put(TurretStates.wanderRight, new WanderRight());
        states.put(TurretStates.manual, new ManualControl());

        currentState = states.get(TurretStates.idle);

        this.turret = turret;
        this.vision = vision;
        this.manualRotationSpeed = manualRotationSpeed;

        ShuffleboardHelper.addOutput("State", Constants.TURRET_TAB, () -> currentState.getClass().getName());
    }

    public static enum TurretStates {
        idle, trackTarget, wanderLeft, wanderRight, manual
    }

    /** Switches the current state, and calls appropriate entry and exit methods. */
    public void setState(TurretStates stateType) {
        turret.resetRotationController();
        SmartDashboard.putString("State", stateType.name());
        TurretState newState = states.get(stateType);

        currentState.onStateExit();

        if (newState == null)
            setState(TurretStates.idle);
        else
            currentState = newState;

        currentState.onStateEntry();
    }

    /** @return the state the turret is currently in */
    public TurretState getCurrentState() {
        return currentState;
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
            setState(TurretStates.manual);
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
        PhotonTrackedTarget target = null;

        @Override
        public void execute() {
            target = vision.getCurrentTarget();

            if (target == null) {
                // Wander in the direction the turret is currently rotating
                // TODO: test if it goes the right direction when it switches to wander
                if (Math.signum(turret.getTurretVelocity()) >= 0)
                    setState(TurretStates.wanderRight);
                else
                    setState(TurretStates.wanderLeft);
                return;
            }

            // Sets the target rotation to the current rotation plus the target's yaw
            turret.setTargetRotation(turret.getCurrentRotation() + target.getYaw());
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
                setState(TurretStates.trackTarget);
                return;
            }

            // If the wander limit is reached, switch directions
            if (currentRotation > wanderLimit) {
                setState(TurretStates.wanderLeft);
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
                setState(TurretStates.trackTarget);
                return;
            }

            // If the wander limit is reached, switch directions
            if (currentRotation < -wanderLimit) {
                setState(TurretStates.wanderRight);
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
                setState(TurretStates.wanderRight);
                return;
            }

            turret.changeTargetRotation(manualSpeed);
        }
    }
}
