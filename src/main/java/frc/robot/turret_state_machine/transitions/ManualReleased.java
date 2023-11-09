package frc.robot.turret_state_machine.transitions;

import frc.robot.Constants;
import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;
import frc.robot.turret_state_machine.Transition;

public class ManualReleased extends Transition<TurretState, TurretData> {

    public ManualReleased(TurretState to) {
        super(to);
    }

    @Override
    public boolean shouldExecute(TurretData turretData) {
        return Math.abs(turretData.manualRotationSpeed) < Constants.AXIS_THRESHOLD;
    }

    @Override
    public TurretState execute(TurretData turretData) {
        return to;
    }
}
