// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.HoodConfig;

public class Hood extends SubsystemBase {
    private final CANSparkMax rotationMotor;
    private final ProfiledPIDController rotationController;

    private double targetRotation, currentRotation;

    public Hood() {
        rotationMotor = new CANSparkMax(CANConfig.HOOD_ROTATION, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();
        rotationMotor.getEncoder().setPositionConversionFactor(HoodConfig.ROTATION_MOTOR_CONVERSION);

        rotationMotor.restoreFactoryDefaults();
        rotationMotor.setInverted(false);

        // Convert to arm rotations in degrees
        rotationMotor.getEncoder().setPositionConversionFactor(HoodConfig.ROTATION_CONVERSION * 360);

        rotationController = new ProfiledPIDController(
                HoodConfig.HOOD_P, HoodConfig.HOOD_I, HoodConfig.HOOD_D,
                new Constraints(HoodConfig.MAX_VELOCITY, HoodConfig.MAX_ACCELERATION));

        ShuffleboardHelper.addOutput("Current", Constants.HOOD_TAB, () -> currentRotation)
                .asDial(-180, 180);
        ShuffleboardHelper.addOutput("Setpoint", Constants.HOOD_TAB, () -> rotationController.getSetpoint().position)
                .asDial(-180, 180);
        ShuffleboardHelper.addOutput("Target", Constants.HOOD_TAB, () -> targetRotation)
                .asDial(-180, 180);

        ShuffleboardHelper.addOutput("Motor Output", Constants.HOOD_TAB, () -> rotationMotor.get());

        // TODO: Remove after initial tuning
        ShuffleboardHelper.addInput("Angle Input", Constants.HOOD_TAB, (value) -> setTargetRotation((double) value),
                getHoodRotation());
        ShuffleboardHelper.addProfiledController("Rotation Controller", Constants.HOOD_TAB, rotationController,
                HoodConfig.MAX_VELOCITY, HoodConfig.MAX_ACCELERATION);
    }

    @Override
    public void periodic() {
        currentRotation = rotationMotor.getEncoder().getPosition();
        updateClosedLoopControl();
    }

    /** Calculates and applies the next output from the PID controller. */
    private void updateClosedLoopControl() {
        double calculatedOutput = rotationController.calculate(getHoodRotation(), targetRotation);
        setTargetRotation(calculatedOutput);
    }

    /** Sets the percent output of the hood rotation motor. */
    public void outputToMotor(double percentOutput) {
        rotationMotor.set(percentOutput);
    }

    /** Returns the hood's current rotation in degrees. */
    public double getHoodRotation() {
        return rotationMotor.getEncoder().getPosition();
    }

    /** Sets the hood's target position. */
    public void setTargetRotation(double targetDegrees) {
        targetRotation = targetDegrees;
    }

    /** Adjusts the hood's target position by the given amount. */
    public void changeTargetRotation(double delta) {
        targetRotation += delta;
    }

    /** Returns the hood's target rotation in degrees. */
    public double getTargetRotation() {
        return targetRotation;
    }
}
