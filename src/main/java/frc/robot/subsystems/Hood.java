// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.HoodConfig;

public class Hood extends SubsystemBase {
    private final CANSparkMax rotationMotor;

    public Hood() {
        rotationMotor = new CANSparkMax(CANConfig.ROTATE_HOOD, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();
        rotationMotor.getEncoder().setPositionConversionFactor(HoodConfig.ROTATION_MOTOR_CONVERSION);
    }

    /** Sets the percent output of the hood rotation motor */
    public void outputToMotor(double percentOutput) {
        rotationMotor.set(percentOutput);
    }

    /** @return the hood current rotation in degrees */
    public double getHoodRotation() {
        return rotationMotor.getEncoder().getPosition();
    }

    private double targetRotation;

    /** Sets the hood actual rotation */
    public void setTargetRotation(double targetDegrees) {
        targetRotation = targetDegrees;
    }

    /** Changes the hood target rotaiton */
    public void changeTargetRotation(double delta) {
        targetRotation += delta;
    }

    /** Getst the hood target rotation */
    public double getTargetRotation() {
        return targetRotation;
    }
}