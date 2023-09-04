// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class RotateTurretManual extends CommandBase {
    private final Turret turret;
    private final Supplier<Double> outputSupplier;

    public RotateTurretManual(Turret turret, Supplier<Double> outputSupplier) {
        this.turret = turret;
        this.outputSupplier = outputSupplier;
    }

    @Override
    public void execute() {
        turret.outputToMotor(outputSupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
        turret.outputToMotor(0);
    }
}
