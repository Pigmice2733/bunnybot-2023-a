// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class EjectAll extends SequentialCommandGroup {
    /** Ejects all balls backward through the indexer and intake */
    public EjectAll(Intake intake, Indexer indexer, Shooter shooter) {
        addCommands(intake.spinBackward(), indexer.spinBeltBackward(),
                shooter.setFlywheelSpeed(AutoConfig.SHOOTER_EJECTION_SPEED),
                Commands.waitSeconds(AutoConfig.EJECT_ALL_TIME),
                intake.stopWheels(), indexer.stopBelt(), shooter.stopFlywheel());

        addRequirements(intake, indexer);
    }
}
