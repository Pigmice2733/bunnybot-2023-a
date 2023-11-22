// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.pigmice.frc.lib.utils.Utils;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.TurretConfig;

public class Turret extends SubsystemBase {
    private final CANSparkMax rotationMotor;
    private final ProfiledPIDController rotationController;

    private double targetRotation;

    public Turret() {
        rotationMotor = new CANSparkMax(CANConfig.ROTATE_TURRET, MotorType.kBrushless);
        rotationMotor.getEncoder().setPosition(0);
        rotationMotor.setInverted(true);

        rotationController = new ProfiledPIDController(
                TurretConfig.ROTATION_P, TurretConfig.ROTATION_I, TurretConfig.ROTATION_D,
                new Constraints(TurretConfig.MAX_VELOCITY, TurretConfig.MAX_ACCELERATION));

        ShuffleboardHelper
                .addOutput("Current Rotation", Constants.TURRET_TAB, () -> getCurrentRotation())
                .asDial(-180, 180);

        ShuffleboardHelper.addOutput("Target Rotation", Constants.TURRET_TAB, () -> targetRotation)
                .asDial(-180, 180);

        ShuffleboardHelper
                .addOutput("Setpoint", Constants.TURRET_TAB,
                        () -> rotationController.getSetpoint().position)
                .asDial(-180, 180);

        ShuffleboardHelper.addOutput("Motor Output", Constants.SHOOTER_TAB,
                () -> rotationMotor.get());

        Constants.TURRET_TAB.add("Reset Encoder",
                new InstantCommand(() -> rotationMotor.getEncoder().setPosition(0)));
    }

    /** Resets the controller to the turret's current rotation. */
    public void resetRotationController() {
        double currentRotation = getCurrentRotation();
        targetRotation = currentRotation;
        rotationController.reset(currentRotation);
    }

    @Override
    public void periodic() {
        updateClosedLoopControl();
    }

    /** Calculates and appliess the next output from the PID controller. */
    private void updateClosedLoopControl() {
        double calculatedOutput = rotationController.calculate(getCurrentRotation(),
                targetRotation);
        outputToMotor(calculatedOutput);
    }

    /** Sets the percent output of the turret rotation motor. */
    public void outputToMotor(double percentOutput) {
        double currentRotation = getCurrentRotation();

        // TODO: assumes (+ output) => (+ rotation). Verify this in later testing.
        percentOutput = Utils.applySoftwareStop(currentRotation, percentOutput,
                TurretConfig.MAX_ALLOWED_ROTATION);

        rotationMotor.set(percentOutput);
    }

    /** Returns the turret's current rotation in degrees. */
    public double getCurrentRotation() {
        // 360 converts rotations to degrees
        // TODO: once design is finalized, figure out the gear ratio and actual sensor
        // conversion and put in constants
        return (rotationMotor.getEncoder().getPosition()) * 360;
    }

    /** Sets the turret's target position. */
    public void setTargetRotation(double targetDegrees) {
        targetRotation = targetDegrees;
    }

    /** Adjusts the turret's target position by the given amount. */
    public void changeTargetRotation(double delta) {
        targetRotation += delta;
    }

    /** Returns the current velocity of the turret in degrees per second. */
    public double getTurretVelocity() {
        return rotationMotor.getEncoder().getVelocity();
    }

    /** Returns the current rotation of the turret in degrees. */
    public double getTargetRotation() {
        return targetRotation;
    }

    /**
     * Sets the max velocity and acceleration of the turret as a Constraints object.
     */
    public void setPIDConstraints(Constraints constraints) {
        rotationController.setConstraints(constraints);
    }
}
