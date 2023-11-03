// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

    /** Manually set the percent output of the motor. */
    public void enableManual(double percentOutput) {
        rotationMotor.set(percentOutput);
    }

    /** Set the motor output to the default speed. */
    public void enableDefault() {
        rotationMotor.set(ShooterConfig.DEFAULT_OUTPUT);
    }

    /** Set the motor to zero output. */
    public void disable() {
        rotationMotor.set(0);
    }

    /** Returns the current speed of the motor. */
    public double getCurrentPercent() {
        return rotationMotor.get();
    }

}
