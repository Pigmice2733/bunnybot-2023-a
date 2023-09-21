package frc.robot;

import java.util.Hashtable;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.TurretConfig;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretStateMachine {
    private Hashtable<Class<?>, TurretState> states = new Hashtable<Class<?>, TurretState>();
    private TurretState currentState;

    private final Vision vision;
    private final Turret turret;

    public TurretStateMachine(Turret turret, Vision vision) {
        states.put(EmptyState.class, new EmptyState());
        states.put(TrackTarget.class, new TrackTarget());
        states.put(WanderLeft.class, new WanderLeft());
        states.put(WanderRight.class, new WanderRight());

        currentState = states.get(EmptyState.class);

        this.turret = turret;
        this.vision = vision;

        ShuffleboardHelper.addOutput("State", Constants.TURRET_TAB, () -> currentState.getClass().getName());
    }

    public void setState(Class<?> stateType) {
        TurretState newState = states.get(stateType);

        if (newState == null)
            return;

        currentState.onStateExit();
        currentState = newState;
        currentState.onStateEntry();
    }

    public TurretState getCurrentState() {
        return currentState;
    }

    public void updateCurrentState() {
        if (currentState == null)
            return;

        currentState.execute();
    }

    public interface TurretState {
        public void onStateEntry();

        public void execute();

        public void onStateExit();
    }

    public class EmptyState implements TurretState {
        @Override
        public void onStateEntry() {
        }

        @Override
        public void execute() {

        }

        @Override
        public void onStateExit() {
        }
    }

    public class TrackTarget implements TurretState {
        @Override
        public void onStateEntry() {
            turret.setPIDConstraints(
                    new Constraints(TurretConfig.MAX_VELOCITY, TurretConfig.MAX_ACCELERATION));
        }

        @Override
        public void execute() {
            PhotonTrackedTarget target = vision.getCurrentTarget();

            if (target == null) {
                // wander in the direction the turret is currently rotating
                if (Math.signum(turret.getTurretVelocity()) >= 0)
                    setState(WanderRight.class);
                else
                    setState(WanderLeft.class); // TODO: test if it goes the right direction when it switches to wander
                return;
            }

            double yaw = target.getYaw();
            turret.setTargetRotation(turret.getCurrentRotation() + yaw);
        }

        @Override
        public void onStateExit() {
        }
    }

    public class WanderLeft implements TurretState {
        double directionMultiplier;
        int wanderLimit = TurretConfig.WANDER_LIMIT;

        @Override
        public void onStateEntry() {
            turret.setPIDConstraints(
                    new Constraints(TurretConfig.MAX_VELOCITY / 4, TurretConfig.MAX_ACCELERATION / 2));

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
                setState(WanderRight.class);
            }
        }

        @Override
        public void onStateExit() {
        }
    }

    public class WanderRight implements TurretState {
        double directionMultiplier;
        int wanderLimit = TurretConfig.WANDER_LIMIT;

        @Override
        public void onStateEntry() {
            turret.setPIDConstraints(
                    new Constraints(TurretConfig.MAX_VELOCITY / 4, TurretConfig.MAX_ACCELERATION / 2));

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

            if (currentRotation < wanderLimit) {
                setState(WanderLeft.class);
            }
        }

        @Override
        public void onStateExit() {
        }
    }
}
