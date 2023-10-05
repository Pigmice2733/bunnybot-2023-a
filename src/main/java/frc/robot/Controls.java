package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DrivetrainConfig;
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

    LinearFilter driveSpeedYFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    /** @return The Left Y Axis multiplied by the drive speed. */
    public double getDriveSpeedY() {
        double joystickValue = driver.getLeftY();
        joystickValue = MathUtil.applyDeadband(-joystickValue, threshold); // deals with stick drag
        // joystickValue = driveSpeedYFilter.calculate(joystickValue); // input
        // smoothing

        return joystickValue * DrivetrainConfig.MAX_DRIVE_SPEED;
    }

    LinearFilter driveSpeedXFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    /** @return The Left X Axis multiplied by the drive speed. */
    public double getDriveSpeedX() {
        double joystickValue = driver.getLeftX();
        joystickValue = MathUtil.applyDeadband(-joystickValue, threshold); // deals with stick drag
        // joystickValue = driveSpeedXFilter.calculate(joystickValue); // input
        // smoothing

        return joystickValue * DrivetrainConfig.MAX_DRIVE_SPEED;
    }

    LinearFilter turnSpeedFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    /** @return The Right X Axis multiplied by the turn speed. */
    public double getTurnSpeed() {
        double joystickValue = driver.getRightX();
        joystickValue = MathUtil.applyDeadband(joystickValue, threshold); // deals with stick drag
        // joystickValue = turnSpeedFilter.calculate(joystickValue); // input smoothing

        return joystickValue * DrivetrainConfig.MAX_TURN_SPEED;
    }

}
