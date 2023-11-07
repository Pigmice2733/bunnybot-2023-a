package frc.robot.turret_state_machine.transitions;

import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;
import frc.robot.turret_state_machine.Transition;

public class VelocityRight extends Transition<TurretState, TurretData> {

    public VelocityRight(TurretState from, TurretState to) {
        super(from, to);
    }

    @Override
    public boolean shouldExecute(TurretData turretData) {
        return turretData.turretVelocity > 0;
    }

    @Override
    public TurretState execute(TurretData turretData) {
        return to;
    }
}
