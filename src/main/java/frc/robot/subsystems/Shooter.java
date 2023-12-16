// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ShooterConfig;

public class Shooter extends SubsystemBase {
    private final CANSparkMax rotationMotor;

    public Shooter() {
        rotationMotor = new CANSparkMax(CANConfig.ROTATE_SHOOTER, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();
        rotationMotor.setInverted(false);
        rotationMotor.setIdleMode(IdleMode.kCoast);

        rotationMotor.getEncoder()
                .setPositionConversionFactor(ShooterConfig.ROTATION_MOTOR_CONVERSION);

        ShuffleboardHelper
                .addOutput("Motor Output", Constants.SHOOTER_TAB, () -> rotationMotor.get())
                .asNumberBar(-1, 1);

        rotationMotor.setSmartCurrentLimit(20);

        outputToMotor(ShooterConfig.IDLE_SPEED);

    }

    /** Manually set the percent output of the motor. */
    public void outputToMotor(double percentOutput) {
        rotationMotor.set(percentOutput > 0 ? 1 : 0);
        rotationMotor.set(percentOutput);
    }

    /** Spins the shooter flywheels at the given speed. */
    public Command setFlywheelSpeed(double percentOutput) {
        return Commands.runOnce(() -> outputToMotor(percentOutput));
    }

    /** Sets the flywheel speed to zero. */
    public Command idleFlywheel() {
        return Commands.runOnce(() -> outputToMotor(ShooterConfig.IDLE_SPEED));
    }

    /** Returns the current speed of the motor. */
    public double getCurrentPercent() {
        return rotationMotor.get();
    }
}
