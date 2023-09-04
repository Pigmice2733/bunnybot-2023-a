// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.TurretConfig;
import frc.robot.subsystems.Turret;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import java.util.function.Supplier;

public class RotateTurretAutomatic extends ProfiledPIDCommand {
    private final Turret turret;
    private final Supplier<Double> manualControl;

    public RotateTurretAutomatic(Turret turret, Supplier<Double> manualControl) {
        super(new ProfiledPIDController(
                TurretConfig.ROTATION_P, TurretConfig.ROTATION_I, TurretConfig.ROTATION_D,
                new Constraints(TurretConfig.MAX_VELOSITY, TurretConfig.MAX_ACCELERATION)),
                () -> turret.getTurretRotation(), () -> new State(turret.getTargetRotation(), 0),
                (output, state) -> turret.outputToMotor(output));

        this.turret = turret;
        this.manualControl = manualControl;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        super.execute();

        turret.changeTargetRotation(manualControl.get());
    }

    @Override
    public void end(boolean interrupted) {
        turret.outputToMotor(0);
    }
}