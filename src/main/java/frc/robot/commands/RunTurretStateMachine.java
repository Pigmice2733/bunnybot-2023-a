// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.turret_state_machine.FiniteStateMachine;
import frc.robot.turret_state_machine.Transition;

public class RunTurretStateMachine extends CommandBase {
    private final FiniteStateMachine<TurretState, TurretData> stateMachine;

    private final Turret turret;
    private final Vision vision;

    private final DoubleSupplier manualRotationSpeed;

    public RunTurretStateMachine(Turret turret, Vision vision, DoubleSupplier manualRotationSpeed) {
        stateMachine = new FiniteStateMachine<TurretState, TurretData>(TurretState.BeginWander,
                new HashMap<TurretState, List<Transition<TurretState, TurretData>>>());

        this.turret = turret;
        this.vision = vision;
        this.manualRotationSpeed = manualRotationSpeed;
    }

    @Override
    public void execute() {
        // PhotonTrackedTarget target = vision.getCurrentTarget();

        // if (target == null) {
        // setState(TurretStates.TRACK_TARGET);
        // return;
        // }

        stateMachine.execute(new TurretData(turret.getTurretVelocity(), manualRotationSpeed.getAsDouble()));
    }

    public enum TurretState {
        BeginWander,
        WanderLeft,
        WanderRight,
        TrackTarget,
        Manual
    }

    public class TurretData {
        public final double turretVelocity;
        public final double manualRotationSpeed;

        public TurretData(double turretVelocity, double manualRotationSpeed) {
            this.turretVelocity = turretVelocity;
            this.manualRotationSpeed = manualRotationSpeed;
        }
    }
}
