// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.turret_state_machine.FiniteStateMachine;
import frc.robot.turret_state_machine.transitions.*;
import frc.robot.turret_state_machine.transitions.running_loops.*;

public class RunTurretStateMachine extends CommandBase {
    private final FiniteStateMachine<TurretState, TurretData> stateMachine;

    private final Turret turret;
    private final Vision vision;
    private final Hood hood;
    private final DoubleSupplier manualRotationSpeed;

    public RunTurretStateMachine(Turret turret, Vision vision, Hood hood, DoubleSupplier manualRotationSpeed) {
        stateMachine = new FiniteStateMachine<TurretState, TurretData>(TurretState.BeginWander);

        stateMachine.addTransitionsFromState(TurretState.BeginWander,
                new ManualPressed(),
                new TargetFound(),
                new VelocityLeft(),
                new VelocityRight());

        stateMachine.addTransitionsFromState(TurretState.WanderLeft,
                new ManualPressed(),
                new TargetFound(),
                new LeftBoundReached(),
                new RunWanderLeft());

        stateMachine.addTransitionsFromState(TurretState.WanderRight,
                new ManualPressed(),
                new TargetFound(),
                new RightBoundReached(),
                new RunWanderRight());

        stateMachine.addTransitionsFromState(TurretState.Manual,
                new ManualReleased(),
                new RunManual());

        stateMachine.addTransitionsFromState(TurretState.TrackTarget,
                new ManualPressed(),
                new TargetLost(),
                new RunTrackTarget());

        this.turret = turret;
        this.vision = vision;
        this.hood = hood;
        this.manualRotationSpeed = manualRotationSpeed;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (!stateMachine.execute(new TurretData(turret, vision, hood, manualRotationSpeed))) {
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
        public final double turretRotation;
        public final double turretVelocity;
        public final double manualRotationSpeed;
        public final boolean hasTarget;
        public final double targetYaw;
        public final double targetArea;
        public final double targetPitch;
        public final DoubleConsumer setTargetRotation;
        public final DoubleConsumer changeTargetRotation;
        public final DoubleConsumer setTargetHoodAngle;
        public final Consumer<Constraints> setTurretConstraints;

        public TurretData(Turret turret, Vision vision, Hood hood, DoubleSupplier rotationSpeed) {
            turretRotation = turret.getCurrentRotation();
            turretVelocity = turret.getTurretVelocity();
            manualRotationSpeed = rotationSpeed.getAsDouble();
            hasTarget = vision.getCurrentTarget() != null;
            targetYaw = hasTarget ? vision.getCurrentTarget().getYaw() : 0;
            targetArea = hasTarget ? vision.getCurrentTarget().getArea() : 0;
            targetPitch = hasTarget ? vision.getCurrentTarget().getPitch() : 0;
            setTargetRotation = turret::setTargetRotation;
            changeTargetRotation = turret::changeTargetRotation;
            setTargetHoodAngle = hood::setTargetRotation;
            setTurretConstraints = turret::setPIDConstraints;
        }
    }
}
