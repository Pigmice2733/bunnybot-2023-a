// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.TurretConfig;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class RotateTurretAutomatic extends ProfiledPIDCommand {
    private final Turret turret;
    private final Vision vision;

    public RotateTurretAutomatic(Turret turret, Vision vision) {
        super(new ProfiledPIDController(
                TurretConfig.ROTATION_P, TurretConfig.ROTATION_I, TurretConfig.ROTATION_D,
                new Constraints(TurretConfig.MAX_VELOCITY, TurretConfig.MAX_ACCELERATION)),
                () -> turret.getCurrentRotation(), () -> new State(turret.getTargetRotation(), 0),
                (output, state) -> turret.outputToMotor(output));

        this.turret = turret;
        this.vision = vision;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        super.execute();

        var yaw = vision.getCurrentTarget().getYaw();
        turret.setTargetRotation(yaw + turret.getCurrentRotation());

        SmartDashboard.putNumber("Vision target yaw", yaw);
        SmartDashboard.putNumber("Current Turret Rotation", turret.getCurrentRotation());
        SmartDashboard.putNumber("Turret Target Rotation", turret.getTargetRotation());
    }

    @Override
    public void end(boolean interrupted) {
        turret.outputToMotor(0);
    }
}
