// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.TurretConfig;

public class Turret extends SubsystemBase {
    private final TalonSRX rotationMotor;

    private double targetRotation;
    private double currentRotation;
    public double setpoint;

    private final GenericEntry motorOutputEntry;

    public Turret() {
        rotationMotor = new TalonSRX(CANConfig.ROTATE_TURRET);
        rotationMotor.setSelectedSensorPosition(0, 0, 0);

        ShuffleboardHelper.addOutput("Current Rotation", Constants.TURRET_TAB, () -> getCurrentRotation())
                .asDial(-360, 360);
        ShuffleboardHelper.addOutput("Target Rotation", Constants.TURRET_TAB, () -> targetRotation)
                .asDial(-360, 360);
        ShuffleboardHelper.addOutput("Setpoint", Constants.TURRET_TAB, () -> setpoint)
                .asDial(-360, 360);

        motorOutputEntry = Constants.TURRET_TAB.add("Motor Output (%)", 0).getEntry();
    }

    @Override
    public void periodic() {
        currentRotation = rotationMotor.getSelectedSensorPosition() / 4096 * 360;
        SmartDashboard.putNumber("Current Rotation", currentRotation);
    }

    /** Sets the percent output of the turret rotation motor */
    public void outputToMotor(double percentOutput) {
        // percentOutput = MathUtil.clamp(percentOutput, -0.1, 0.1);
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

    /** Getst the turrets target rotation */
    public double getTargetRotation() {
        return targetRotation;
    }
}
