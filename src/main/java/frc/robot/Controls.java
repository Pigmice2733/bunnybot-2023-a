package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controls {

    XboxController driver;
    XboxController operator;

    private double threshold = Constants.AXIS_THRESHOLD; // If a value from a joystick is less than this, it will return
                                                         // 0.

    public Controls(XboxController driver, XboxController operator) {
        this.driver = driver;
        this.operator = operator;
    }
}
