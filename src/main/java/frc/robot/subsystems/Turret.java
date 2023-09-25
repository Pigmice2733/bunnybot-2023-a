// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.TurretConfig;

public class Turret extends SubsystemBase {
    private final TalonSRX rotationMotor;

    private double targetRotation;
    private double currentRotation;

    private final GenericEntry motorOutputEntry;

    private final ProfiledPIDController rotationController;
    private Constraints constraints;

    public Turret() {
        rotationMotor = new TalonSRX(CANConfig.ROTATE_TURRET);
        rotationMotor.setSelectedSensorPosition(0, 0, 0);

        constraints = new Constraints(TurretConfig.MAX_VELOCITY, TurretConfig.MAX_ACCELERATION);

        rotationController = new ProfiledPIDController(
                TurretConfig.ROTATION_P, TurretConfig.ROTATION_I, TurretConfig.ROTATION_D,
                constraints);

        rotationController.enableContinuousInput(-180, 180);

        ShuffleboardHelper.addOutput("Current Rotation", Constants.TURRET_TAB, () -> getCurrentRotation())
                .asDial(-180, 180);

        ShuffleboardHelper.addOutput("Target Rotation", Constants.TURRET_TAB, () -> targetRotation)
                .asDial(-180, 180);

        ShuffleboardHelper.addOutput("Setpoint", Constants.TURRET_TAB, () -> rotationController.getSetpoint().position)
                .asDial(-180, 180);

        motorOutputEntry = Constants.TURRET_TAB.add("Motor Output (%)", 0).getEntry();
    }

    @Override
    public void periodic() {
        updateClosedLoopControl();
    }

    /** Calculates and applys the next output from the PID controller */
    private void updateClosedLoopControl() {
        currentRotation = getCurrentRotation();
        double calculatedOutput = rotationController.calculate(currentRotation, targetRotation);

        outputToMotor(calculatedOutput);
    }

    /** Sets the percent output of the turret rotation motor */
    public void outputToMotor(double percentOutput) {
        rotationMotor.set(TalonSRXControlMode.PercentOutput, percentOutput);
        motorOutputEntry.setDouble(percentOutput);
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

    public double getTurretVelocity() {
        return rotationMotor.getSelectedSensorVelocity();
    }

    /** Getst the turrets target rotation */
    public double getTargetRotation() {
        return targetRotation;
    }

    public void setPIDConstraints(Constraints constraints) {
        this.constraints = constraints;
    }
}
