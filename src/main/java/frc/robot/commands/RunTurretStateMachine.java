// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TurretStateMachine;
import frc.robot.TurretStateMachine.TurretStates;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class RunTurretStateMachine extends CommandBase {
    private final TurretStateMachine stateMachine;

    public RunTurretStateMachine(Turret turret, Vision vision, DoubleSupplier manualRotationSpeed) {
        this.stateMachine = new TurretStateMachine(turret, vision, manualRotationSpeed);
        addRequirements(turret, vision);
    }

    @Override
    public void initialize() {
        stateMachine.setState(TurretStates.wanderRight);
    }

    @Override
    public void execute() {
        stateMachine.updateStateMachine();
    }
}
