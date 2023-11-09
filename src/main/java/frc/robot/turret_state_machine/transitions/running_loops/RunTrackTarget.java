package frc.robot.turret_state_machine.transitions.running_loops;

import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;

public class RunTrackTarget extends RunningLoop<TurretState, TurretData> {

    public RunTrackTarget() {
        super(TurretState.TrackTarget);
    }

    @Override
    protected void run(TurretData turretData) {
        turretData.setTargetRotation.accept(turretData.turretRotation + turretData.targetYaw);
    }
}
