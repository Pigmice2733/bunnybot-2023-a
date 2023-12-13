// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControllerRumbler;
import frc.robot.Constants.GrabberConfig;
import frc.robot.Constants.GrabberConfig.ArmPosition;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;

public class ZeroGrabber extends CommandBase {
  private final Grabber grabber;
  private final Intake intake;

  public ZeroGrabber(Grabber grabber, Intake intake) {
    this.grabber = grabber;
    this.intake = intake;

    addRequirements(grabber);
  }

  @Override
  public void initialize() {
    grabber.stopPID();
    intake.stopWheels();
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
      grabber.resetPID();
    } else {
      grabber.resetPID();
      grabber.setTargetRotation(grabber.getCurrentRotation());
    }

    grabber.startPID();
    grabber.setTargetArmAngleCommand(ArmPosition.STOW).schedule();

    intake.spinForward();
  }

  @Override
  public boolean isFinished() {
    return grabber.limitSwitchPressed();
  }
}
