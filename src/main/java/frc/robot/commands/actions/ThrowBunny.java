// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Grabber;

public class ThrowBunny extends SequentialCommandGroup {
  public ThrowBunny(SwerveDrivetrain drivetrain, Grabber grabber) {
    addRequirements(drivetrain, grabber);
    addCommands(null);
  }
}
