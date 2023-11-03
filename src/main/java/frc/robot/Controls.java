package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.TurretConfig;

public class Controls {

    XboxController driver;
    XboxController operator;

    // If a value from a joystick is less than this, it will return 0.
    private double threshold = Constants.AXIS_THRESHOLD;

    private double joystickTurret, joystickY, joystickX, joystickTurn;

    public Controls(XboxController driver, XboxController operator) {
        this.driver = driver;
        this.operator = operator;
    }

    /**
     * Returns the controller input multiplied by the drive speed. The right trigger
     * moves in the positive direction, and the left moves in the negative
     * direction, so controller input is defined as right minus left.
     */
    public double getManualTurretRotationSpeed() {
        joystickTurret = MathUtil.applyDeadband(operator.getRightTriggerAxis(), threshold) -
                MathUtil.applyDeadband(operator.getLeftTriggerAxis(), threshold);
        return joystickTurret * TurretConfig.MANUAL_ROTATION_MULTIPLIER;
    }

    LinearFilter driveSpeedYFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    /** Returns the left joystick y-axis multiplied by the drive speed. */
    public double getDriveSpeedY() {
        joystickY = MathUtil.applyDeadband(-driver.getLeftY(), threshold);
        // joystickValue = driveSpeedYFilter.calculate(joystickValue); // input
        // smoothing
        return joystickY * DrivetrainConfig.MAX_DRIVE_SPEED;
    }

    LinearFilter driveSpeedXFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    /** Returns the left joystick x-axis multiplied by the drive speed. */
    public double getDriveSpeedX() {
        joystickX = MathUtil.applyDeadband(-driver.getLeftX(), threshold);
        // joystickValue = driveSpeedXFilter.calculate(joystickValue); // input
        // smoothing
        return joystickX * DrivetrainConfig.MAX_DRIVE_SPEED;
    }

    LinearFilter turnSpeedFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    /** Returns the right joystick x-axis multiplied by the drive speed. */
    public double getTurnSpeed() {
        joystickTurn = MathUtil.applyDeadband(driver.getRightX(), threshold);
        // joystickValue = turnSpeedFilter.calculate(joystickValue); // input smoothing
        return joystickTurn * DrivetrainConfig.MAX_TURN_SPEED;
    }

}
