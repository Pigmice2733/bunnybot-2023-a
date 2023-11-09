// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.turret_state_machine.FiniteStateMachine;
import frc.robot.turret_state_machine.transitions.LeftBoundReached;
import frc.robot.turret_state_machine.transitions.ManualPressed;
import frc.robot.turret_state_machine.transitions.ManualReleased;
import frc.robot.turret_state_machine.transitions.TargetFound;
import frc.robot.turret_state_machine.transitions.TargetLost;
import frc.robot.turret_state_machine.transitions.VelocityLeft;
import frc.robot.turret_state_machine.transitions.VelocityRight;
import frc.robot.turret_state_machine.transitions.running_loops.RunManual;
import frc.robot.turret_state_machine.transitions.running_loops.RunTrackTarget;
import frc.robot.turret_state_machine.transitions.running_loops.RunWanderLeft;
import frc.robot.turret_state_machine.transitions.running_loops.RunWanderRight;

public class RunTurretStateMachine extends CommandBase {
    private final FiniteStateMachine<TurretState, TurretData> stateMachine;

    private final Turret turret;
    private final Vision vision;
    private final DoubleSupplier manualRotationSpeed;
    private boolean hasTarget;

    public RunTurretStateMachine(Turret turret, Vision vision, DoubleSupplier manualRotationSpeed) {
        stateMachine = new FiniteStateMachine<TurretState, TurretData>(TurretState.BeginWander);

        stateMachine.addTransitionsFromState(TurretState.BeginWander,
                new ManualPressed(),
                new TargetFound(TurretState.TrackTarget),
                new VelocityLeft(TurretState.WanderLeft),
                new VelocityRight(TurretState.WanderRight));

        stateMachine.addTransitionsFromState(TurretState.WanderLeft,
                new ManualPressed(),
                new TargetFound(TurretState.TrackTarget),
                new LeftBoundReached(TurretState.WanderRight),
                new RunWanderLeft());

        stateMachine.addTransitionsFromState(TurretState.WanderRight,
                new ManualPressed(),
                new TargetFound(TurretState.TrackTarget),
                new LeftBoundReached(TurretState.WanderLeft),
                new RunWanderRight());

        stateMachine.addTransitionsFromState(TurretState.Manual,
                new ManualReleased(TurretState.BeginWander),
                new TargetLost(TurretState.BeginWander),
                new RunManual());

        stateMachine.addTransitionsFromState(TurretState.TrackTarget,
                new ManualPressed(),
                new RunTrackTarget());

        this.turret = turret;
        this.vision = vision;
        this.manualRotationSpeed = manualRotationSpeed;

        addRequirements(turret, vision);
    }

    @Override
    public void execute() {
        hasTarget = vision.getCurrentTarget() != null;

        if (!stateMachine.execute(new TurretData(
                turret.getCurrentRotation(),
                turret.getTurretVelocity(),
                manualRotationSpeed.getAsDouble(),
                hasTarget,
                hasTarget ? vision.getCurrentTarget().getYaw() : 0,
                hasTarget ? vision.getCurrentTarget().getPitch() : 0,
                turret,
                (value) -> turret.setTargetRotation(value),
                (value) -> turret.changeTargetRotation(value)))) {
            System.out.println("Turret state machine encountered an error.");
        }
    }

    public enum TurretState {
        BeginWander,
        WanderLeft,
        WanderRight,
        TrackTarget,
        Manual
    }

    public class TurretData {
        public final Turret turret;
        public final double turretRotation;
        public final double turretVelocity;
        public final double manualRotationSpeed;
        public final boolean hasTarget;
        public final double targetYaw;
        public final double targetPitch;
        public final DoubleConsumer setTargetRotation;
        public final DoubleConsumer changeTargetRotation;

        public TurretData(double turretRotation, double turretVelocity, double manualRotationSpeed,
                boolean hasTarget, double targetYaw, double targetPitch, Turret turret,
                DoubleConsumer setTargetRotation, DoubleConsumer changeTargetRotation) {

            this.turret = turret;
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
