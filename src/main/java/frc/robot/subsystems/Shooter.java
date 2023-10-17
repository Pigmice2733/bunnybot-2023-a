package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ShooterConfig;

public class Shooter extends SubsystemBase {
    private final CANSparkMax rotationMotor;

    public Shooter() {
        rotationMotor = new CANSparkMax(CANConfig.ROTATE_SHOOTER, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();
        rotationMotor.getEncoder().setPositionConversionFactor(ShooterConfig.ROTATION_MOTOR_CONVERSION);

        ShuffleboardHelper.addOutput("Current Percent", Constants.SHOOTER_TAB, () -> getCurrentPercent())
                .asDial(-1, 1);
    }

    public void enableManual(double percentOutput) {
        rotationMotor.set(percentOutput);
    }

    public void enableDefault() {
        rotationMotor.set(ShooterConfig.DEFAULT_OUTPUT);
    }

    public void disable() {
        rotationMotor.set(0);
    }

    public double getCurrentPercent() {
        return rotationMotor.get();
    }

}
