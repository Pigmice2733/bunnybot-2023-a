package frc.robot.turret_state_machine.transitions.running_loops;

import edu.wpi.first.math.MathUtil;
import frc.robot.commands.RunTurretStateMachine.TurretData;
import frc.robot.commands.RunTurretStateMachine.TurretState;

public class RunTrackTarget extends RunningLoop<TurretState, TurretData> {

    /** Automatically track the best target using vision. */
    public RunTrackTarget() {
        super(TurretState.TrackTarget);
    }

    @Override
    protected void run(TurretData turretData) {
        turretData.setTargetRotation.accept(turretData.turretRotation + turretData.targetYaw);
        turretData.setTargetHoodAngle.accept(calculateHoodAngle(turretData.targetArea * 2.0));
    }

    private double calculateHoodAngle(double targetArea) {
        if (targetArea == 0)
            return 10;

        // double angle = 79.6 - 59.1 * targetArea + 6.16 * targetArea * targetArea;
        double angle = 73.4 - 87.1 * targetArea + 23.7 * targetArea * targetArea;
        angle = MathUtil.clamp(angle, 0, 95);
        return angle;
    }
}
