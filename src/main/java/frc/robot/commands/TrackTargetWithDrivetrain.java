// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;

public class TrackTargetWithDrivetrain extends ProfiledPIDCommand {

  public TrackTargetWithDrivetrain(SwerveDrivetrain drivetrain, Vision vision) {
    super(
        new ProfiledPIDController(0.003, 0, 0, new Constraints(0, 0)),
        () -> {
          return vision.getCurrentTarget() == null
              ? 0
              : vision.getCurrentTarget().getYaw();
        },
        () -> new State(),
        (output, state) -> drivetrain.driveChassisSpeeds(
            new ChassisSpeeds(0, 0, output)),
        drivetrain);

    getController().setTolerance(8);

    ShuffleboardHelper.addProfiledController("Auto aim", Constants.VISION_TAB, getController(), 0, 0);

    addRequirements(drivetrain, vision);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
