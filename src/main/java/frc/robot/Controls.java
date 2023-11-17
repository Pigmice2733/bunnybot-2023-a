package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.HoodConfig;
import frc.robot.Constants.TurretConfig;

public class Controls {

    XboxController driver;
    XboxController operator;

    // If a value from a joystick is less than this, it will return 0.
    private double threshold = Constants.AXIS_THRESHOLD;

    private double joystickTurret, joystickY, joystickX, joystickTurn, joystickHood;

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
        return joystickY * DrivetrainConfig.MAX_DRIVE_SPEED
                * (driver.getYButton() ? DrivetrainConfig.SLOWMODE_MULTIPLIER : 1);
    }

    LinearFilter driveSpeedXFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    /**
     * Returns the left joystick x-axis multiplied by the drive speed. When the Y
     * button is held, the result is multiplied by the slowmode multiplier before
     * returning.
     */
    public double getDriveSpeedX() {
        joystickX = MathUtil.applyDeadband(-driver.getLeftX(), threshold);
        // joystickValue = driveSpeedXFilter.calculate(joystickValue); // input
        // smoothing
        return joystickX * DrivetrainConfig.MAX_DRIVE_SPEED
                * (driver.getYButton() ? DrivetrainConfig.SLOWMODE_MULTIPLIER : 1);
    }

    LinearFilter turnSpeedFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    /**
     * Returns the right joystick x-axis multiplied by the drive speed. When the Y
     * button is held, the result is multiplied by the slowmode multiplier before
     * returning.
     */
    public double getTurnSpeed() {
        joystickTurn = MathUtil.applyDeadband(driver.getRightX(), threshold);
        // joystickValue = turnSpeedFilter.calculate(joystickValue); // input smoothing
        return joystickTurn * DrivetrainConfig.MAX_TURN_SPEED;
    }

    public double getManualHoodSpeed() {
        joystickHood = MathUtil.applyDeadband(operator.getLeftY(), threshold);
        return joystickHood * HoodConfig.MANUAL_ROTATION_SPEED;
    }
}
