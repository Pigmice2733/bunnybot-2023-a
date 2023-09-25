package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.TurretConfig;

public class Controls {

    XboxController driver;
    XboxController operator;

    private double threshold = Constants.AXIS_THRESHOLD; // If a value from a joystick is less than this, it will return
                                                         // 0.

    public Controls(XboxController driver, XboxController operator) {
        this.driver = driver;
        this.operator = operator;
    }

    public double getManualTurretRotationSpeed() {
        double joystickValue = operator.getRightTriggerAxis() - operator.getLeftTriggerAxis();
        joystickValue = MathUtil.applyDeadband(joystickValue, threshold);

        return Math.signum(joystickValue) * TurretConfig.MANUAL_ROTATION_SPEED;
    }
}
