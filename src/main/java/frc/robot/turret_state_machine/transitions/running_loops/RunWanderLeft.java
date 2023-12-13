package frc.robot.turret_state_machine.transitions.running_loops;

import frc.robot.Constants.TurretConfig;
import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;

public class RunWanderLeft extends RunningLoop<TurretState, TurretData> {

    /** Rotate the turret left to scan for targets. */
    public RunWanderLeft() {
        super(TurretState.WanderLeft);
    }

    @Override
    protected void run(TurretData turretData) {
        // turretData.setTurretConstraints
        //         .accept(new Constraints(TurretConfig.WANDER_VEL, TurretConfig.MAX_ACCELERATION));
        turretData.setTargetRotation.accept(-TurretConfig.WANDER_LIMIT - 5);
    }
}
