// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
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
        double turretRotation = turret.getCurrentRotation();
        double turretVelocity = turret.getTurretVelocity();
        double currentManualSpeed = manualRotationSpeed.getAsDouble();

        PhotonTrackedTarget target = vision.getCurrentTarget();
        boolean hasTarget = target == null;

        double targetYaw = hasTarget ? target.getYaw() : 0;
        double targetPitch = hasTarget ? target.getPitch() : 0;

        Consumer<Double> setTargetRotation = (value) -> turret.setTargetRotation(value);
        Consumer<Double> changeTargetRotation = (value) -> turret.changeTargetRotation(value);

        TurretData turretData = new TurretData(turretRotation, turretVelocity, currentManualSpeed, hasTarget, targetYaw,
                targetPitch, turret, setTargetRotation, changeTargetRotation);

        stateMachine.execute(turretData);
    }

    public enum TurretState {
        BeginWander,
        WanderLeft,
        WanderRight,
        TrackTarget,
        Manual
    }

    public class TurretData {
        public final double turretRotation;
        public final double turretVelocity;
        public final double manualRotationSpeed;
        public final boolean hasTarget;
        public final double targetYaw;
        public final double targetPitch;

        public final Consumer<Double> setTargetRotation;
        public final Consumer<Double> changeTargetRotation;

        public TurretData(double turretRotation, double turretVelocity, double manualRotationSpeed, boolean hasTarget,
                double targetYaw, double targetPitch, Turret turret,
                Consumer<Double> setTargetRotation, Consumer<Double> changeTargetRotation) {

            this.turretRotation = turretRotation;
            this.turretVelocity = turretVelocity;
            this.manualRotationSpeed = manualRotationSpeed;
            this.hasTarget = hasTarget;
            this.targetYaw = targetYaw;
            this.targetPitch = targetPitch;

            this.setTargetRotation = setTargetRotation;
            this.changeTargetRotation = changeTargetRotation;
        }
    }
}
