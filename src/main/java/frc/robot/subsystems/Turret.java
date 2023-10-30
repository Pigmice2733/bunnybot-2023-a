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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.TurretConfig;

public class Turret extends SubsystemBase {
    private final TalonSRX rotationMotor; // TODO: when ready to test on the actual robot, switch to a CANSparkMax
    private final ProfiledPIDController rotationController;
    private final GenericEntry motorOutputEntry;

    private double targetRotation;

    public Turret() {
        rotationMotor = new TalonSRX(CANConfig.ROTATE_TURRET);
        rotationMotor.setSelectedSensorPosition(0, 0, 0);
        rotationMotor.setInverted(false);

        rotationController = new ProfiledPIDController(
                TurretConfig.ROTATION_P, TurretConfig.ROTATION_I, TurretConfig.ROTATION_D,
                new Constraints(TurretConfig.MAX_VELOCITY, TurretConfig.MAX_ACCELERATION));

        ShuffleboardHelper.addOutput("Current Rotation", Constants.TURRET_TAB, () -> getCurrentRotation())
                .asDial(-180, 180);

        ShuffleboardHelper.addOutput("Target Rotation", Constants.TURRET_TAB, () -> targetRotation)
                .asDial(-180, 180);

        ShuffleboardHelper.addOutput("Setpoint", Constants.TURRET_TAB, () -> rotationController.getSetpoint().position)
                .asDial(-180, 180);

        motorOutputEntry = Constants.TURRET_TAB.add("Motor Output (%)", 0).getEntry();

        Constants.TURRET_TAB.add("Reset Encoder", new InstantCommand(() -> rotationMotor.setSelectedSensorPosition(0)));
    }

    public void resetRotationController() {
        double currentRotation = getCurrentRotation();
        targetRotation = currentRotation;
        rotationController.reset(currentRotation);
    }

    @Override
    public void periodic() {
        updateClosedLoopControl();
    }

    /** Calculates and applys the next output from the PID controller */
    private void updateClosedLoopControl() {
        double calculatedOutput = rotationController.calculate(getCurrentRotation(), targetRotation);
        outputToMotor(calculatedOutput);
    }

    /** Sets the percent output of the turret rotation motor */
    public void outputToMotor(double percentOutput) {
        double currentRotation = getCurrentRotation();

        // TODO: assumes (+ output) => (+ rotation). Verify this in later testing
        if (currentRotation > TurretConfig.MAX_ALLOWED_ROTATION)
            percentOutput = Math.min(0, percentOutput);
        if (currentRotation < -TurretConfig.MAX_ALLOWED_ROTATION)
            percentOutput = Math.max(0, percentOutput);

        rotationMotor.set(TalonSRXControlMode.PercentOutput, -percentOutput);
        motorOutputEntry.setDouble(percentOutput / 10);
    }

    /** @return the turrets current rotation in degrees */
    public double getCurrentRotation() {
        // 4090 is sensor ticks per rotation, and 360 converts rotations to degrees
        // TODO: once design is finalized, figure out the gear ratio and actual sensor
        // conversion and put in constants
        return (rotationMotor.getSelectedSensorPosition() / 4096) * 360;
    }

    /** Sets the turrets actual rotation */
    public void setTargetRotation(double targetDegrees) {
        targetRotation = targetDegrees;
    }

    /** Changes the turrets target rotaiton */
    public void changeTargetRotation(double delta) {
        targetRotation += delta;
    }

    /** @return the current velocity of the turret in degrees / sec */
    public double getTurretVelocity() {
        return rotationMotor.getSelectedSensorVelocity();
    }

    /** @return the current rotation of the turret in degrees */
    public double getTargetRotation() {
        return targetRotation;
    }

    /** Sets the velocity and acceleration of the turret */
    public void setPIDConstraints(Constraints constraints) {
        rotationController.setConstraints(constraints);
    }

    // TODO: return true if the turret is aimed at a targeted at an ok velocity
    public boolean hasTarget() {
        return false;
    }
}
