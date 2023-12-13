// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControllerRumbler;
import frc.robot.Constants.GrabberConfig;
import frc.robot.Constants.HoodConfig;
import frc.robot.Constants.GrabberConfig.ArmPosition;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Hood;

public class ZeroHood extends CommandBase {
  private final Hood hood;

  public ZeroHood(Hood hood) {
    this.hood = hood;

    addRequirements(hood);
  }

  @Override
  public void initialize() {
    hood.stopPID();
  }

  @Override
  public void execute() {
    hood.outputToMotor(-HoodConfig.MOTOR_ZERO_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      hood.setEncoderPosition(0);
      ControllerRumbler.rumblerOperator(RumbleType.kBothRumble, 0.3, 0.7);
    } else {
      hood.setTargetRotation(hood.getCurrentRotation());
    }

    hood.resetPID();
    hood.startPID();
  }

  @Override
  public boolean isFinished() {
    System.out.println(hood.getVelocity());
    // return hood.limitSwitchPressed();
    return false;
  }
}
