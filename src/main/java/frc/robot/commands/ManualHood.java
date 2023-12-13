// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class ManualHood extends CommandBase {
    private final Hood hood;
    private final DoubleSupplier manualRotationSpeed;

    public ManualHood(Hood hood, DoubleSupplier manualRotationSpeed) {
        this.hood = hood;
        this.manualRotationSpeed = manualRotationSpeed;

        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setTargetRotation(manualRotationSpeed.getAsDouble());
    }
}
