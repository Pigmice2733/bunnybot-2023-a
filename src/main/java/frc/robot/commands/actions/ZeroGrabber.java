// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControllerRumbler;
import frc.robot.Constants.GrabberConfig;
import frc.robot.subsystems.Grabber;

public class ZeroGrabber extends CommandBase {
  private final Grabber grabber;

  public ZeroGrabber(Grabber grabber) {
    this.grabber = grabber;

    addRequirements(grabber);
  }

  @Override
  public void initialize() {
    grabber.runPID = false;
  }

  @Override
  public void execute() {
    grabber.outputToRotationMotor(-GrabberConfig.MOTOR_ZERO_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      grabber.setEncoderPosition(0);
      ControllerRumbler.rumblerOperator(RumbleType.kBothRumble, 0.3, 0.7);
    } else {
      grabber.setTargetRotation(grabber.getCurrentRotation());
    }

    grabber.resetPID();

    grabber.runPID = true;
  }

  @Override
  public boolean isFinished() {
    return grabber.limitSwitchPressed();
  }
}
