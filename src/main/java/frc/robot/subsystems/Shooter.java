// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ShooterConfig;

public class Shooter extends SubsystemBase {
    private final CANSparkMax rotationMotor;

    public Shooter() {
        rotationMotor = new CANSparkMax(CANConfig.ROTATE_SHOOTER, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();
        rotationMotor.getEncoder().setPositionConversionFactor(ShooterConfig.ROTATION_MOTOR_CONVERSION);
    }

    /** Sets the percent output of the shooter rotation motor */
    public void outputToMotor(double percentOutput) {
        rotationMotor.set(percentOutput);
    }

    private double targetRotation;

    /** Sets the turrets actual rotation */
    public void setTargetRotation(double targetDegrees) {
        targetRotation = targetDegrees;
    }

    /** Changes the turrets target rotaiton */
    public void changeTargetRotation(double delta) {
        targetRotation += delta;
    }

    /** Getst the turrets target rotation */
    public double getTargetRotation() {
        return targetRotation;
    }
}
