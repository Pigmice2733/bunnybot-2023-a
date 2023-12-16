// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class PauseTurret extends CommandBase {
  /** Creates a new PauseTurret. */
  public PauseTurret(Turret turret) {
    addRequirements(turret);
  }
}
