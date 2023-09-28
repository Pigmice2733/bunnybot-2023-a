package frc.robot;

import java.util.Hashtable;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TurretConfig;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretStateMachine {
    private Hashtable<Class<?>, TurretState> states = new Hashtable<Class<?>, TurretState>();
    private TurretState currentState;

    private final Vision vision;
    private final Turret turret;

    private final Supplier<Double> manualRotationSpeed;

    /** A state machine to control the turret's behaviour */
    public TurretStateMachine(Turret turret, Vision vision, Supplier<Double> manualRotationSpeed) {
        states.put(Idle.class, new Idle());
        states.put(TrackTarget.class, new TrackTarget());
        states.put(WanderLeft.class, new WanderLeft());
        states.put(WanderRight.class, new WanderRight());
        states.put(ManualControl.class, new ManualControl());

        currentState = states.get(Idle.class);

        this.turret = turret;
        this.vision = vision;
        this.manualRotationSpeed = manualRotationSpeed;

        // ShuffleboardHelper.addOutput("State", Constants.TURRET_TAB, () ->
        // currentState.getClass().getName());
    }

    /** Switches the current state, and calls appropriate entry and exit methods */
    public void setState(Class<?> stateType) {
        turret.resetRotationController();
        SmartDashboard.putString("State", stateType.getName());
        TurretState newState = states.get(stateType);

        if (newState == null)
            setState(Idle.class);

        currentState.onStateExit();
        currentState = newState;
        currentState.onStateEntry();
    }

    /** @return the state the turret is currently in */
    public TurretState getCurrentState() {
        return currentState;
    }

    /** Periodically updates the state macine, including the curret state */
    public void updateStateMachine() {
        if (currentState == null)
            return;

        if (Math.abs(manualRotationSpeed.get()) > Constants.AXIS_THRESHOLD) {
            setState(ManualControl.class);
        }

        currentState.execute();
    }

    /** Implement this for any behaviour of the turret */
    public interface TurretState {
        /** Called once when the turret enters this state */
        public default void onStateEntry() {
        };

        /** Called repeatedly while the turret is in this state */
        public default void execute() {
        };

        /** Called once when the turret exits this state */
        public default void onStateExit() {
        };
    }

    /** When the turret is idle, it will not move at all */
    public class Idle implements TurretState {
    }

    /** Tracks a target until it is not seen, then switches to wander */
    public class TrackTarget implements TurretState {
        @Override
        public void execute() {
            PhotonTrackedTarget target = vision.getCurrentTarget();

            if (target == null) {
                // wander in the direction the turret is currently rotating
                // TODO: test if it goes the right direction when it switches to wander
                if (Math.signum(turret.getTurretVelocity()) >= 0)
                    setState(WanderRight.class);
                else
                    setState(WanderRight.class);
                return;
            }

            // Sets the target rotation to the current rotation plus the target's yaw
            double yaw = target.getYaw();
            turret.setTargetRotation(turret.getCurrentRotation() + yaw);
        }
    }

    /** Rotates left until a target is found, or the wander boundary is reached */
    public class WanderRight implements TurretState {
        double directionMultiplier;
        double wanderLimit = TurretConfig.WANDER_LIMIT;

        @Override
        public void onStateEntry() {
            turret.setPIDConstraints(
                    new Constraints(TurretConfig.WANDER_VELOCITY, TurretConfig.WANDER_ACCELERATION));

            turret.setTargetRotation(wanderLimit + 5);
        }

        @Override
        public void execute() {
            PhotonTrackedTarget target = vision.getCurrentTarget();
            double currentRotation = turret.getCurrentRotation();

            if (target != null) {
                setState(TrackTarget.class);
                return;
            }

            if (currentRotation > wanderLimit) {
                setState(WanderLeft.class);
            }
        }

        @Override
        public void onStateExit() {
            turret.setPIDConstraints(
                    new Constraints(TurretConfig.MAX_VELOCITY, TurretConfig.MAX_ACCELERATION));
        }
    }

    /** Rotates right until a target is found, or the wander boundary is reached */
    public class WanderLeft implements TurretState {
        double directionMultiplier;
        double wanderLimit = TurretConfig.WANDER_LIMIT;

        @Override
        public void onStateEntry() {
            turret.setPIDConstraints(
                    new Constraints(TurretConfig.WANDER_VELOCITY, TurretConfig.WANDER_ACCELERATION));

            turret.setTargetRotation(-wanderLimit - 5);
        }

        @Override
        public void execute() {
            PhotonTrackedTarget target = vision.getCurrentTarget();
            double currentRotation = turret.getCurrentRotation();

            if (target != null) {
                setState(TrackTarget.class);
                return;
            }

            if (currentRotation < -wanderLimit) {
                setState(WanderRight.class);
            }
        }

        @Override
        public void onStateExit() {
            turret.setPIDConstraints(
                    new Constraints(TurretConfig.MAX_VELOCITY, TurretConfig.MAX_ACCELERATION));
        }
    }

    /** Manually controls the turret until the input is zero, then wander */
    public class ManualControl implements TurretState {
        @Override
        public void execute() {
            double speed = manualRotationSpeed.get();
            if (Math.abs(speed) < Constants.AXIS_THRESHOLD) {
                setState(WanderRight.class);
                return;
            }

            turret.changeTargetRotation(speed);
        }
    }
}
