package frc.robot.turret_state_machine.transitions.running_loops;

import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;

public class RunManual extends RunningLoop<TurretState, TurretData> {

    /** Manual control of the turret using controller triggers. */
    public RunManual() {
        super(TurretState.Manual);
    }

    @Override
    protected void run(TurretData turretData) {
        turretData.changeTargetRotation.accept(turretData.manualRotationSpeed);
    }
}
