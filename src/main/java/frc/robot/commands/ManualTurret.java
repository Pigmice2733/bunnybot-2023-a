// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ManualTurret extends CommandBase {
    private final Turret turret;
    private final DoubleSupplier manualRotationSpeed;

    public ManualTurret(Turret turret, DoubleSupplier manualRotationSpeed) {
        this.turret = turret;
        this.manualRotationSpeed = manualRotationSpeed;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.setTargetRotation(turret.getCurrentRotation() + manualRotationSpeed.getAsDouble());
    }
}
