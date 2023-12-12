// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControllerRumbler;
import frc.robot.Constants.TurretConfig;
import frc.robot.subsystems.Turret;

public class ZeroTurret extends CommandBase {
  private final Turret turret;

  public ZeroTurret(Turret turret) {
    this.turret = turret;

    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.runPID = false;
  }

  @Override
  public void execute() {
    turret.outputToMotor(-TurretConfig.ZERO_TURRET_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      turret.setEncoderPosition(0);
      ControllerRumbler.rumblerOperator(RumbleType.kBothRumble, 0.3, 0.7);
    } else {
      turret.setTargetRotation(turret.getCurrentRotation());
    }

    turret.resetPID();

    turret.runPID = true;
  }

  @Override
  public boolean isFinished() {
    return turret.limitSwitchPressed();
  }
}
