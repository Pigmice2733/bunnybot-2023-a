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
import frc.robot.Constants.TurretConfig;

public class Turret extends SubsystemBase {
    private final CANSparkMax rotationMotor;

    private double targetRotation;
    private double currentRotation;

    public Turret() {
        rotationMotor = new CANSparkMax(CANConfig.ROTATE_TURRET, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();
        rotationMotor.getEncoder().setPositionConversionFactor(TurretConfig.ROTATION_MOTOR_CONVERSION);

        ShuffleboardHelper.addOutput("Current Rotation", Constants.TURRET_TAB, () -> getCurrentRotation())
                .asDial(0, 360);
        ShuffleboardHelper.addOutput("Target Rotation", Constants.TURRET_TAB, () -> targetRotation)
                .asDial(0, 360);
    }

    @Override
    public void periodic() {
        currentRotation = rotationMotor.getEncoder().getPosition();
    }

    /** Sets the percent output of the turret rotation motor */
    public void outputToMotor(double percentOutput) {
        rotationMotor.set(percentOutput);
    }

    /** @return the turrets current rotation in degrees */
    public double getCurrentRotation() {
        return currentRotation;
    }

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
