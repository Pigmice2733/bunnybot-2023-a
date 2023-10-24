package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.IndexerConfig;

public class Indexer extends SubsystemBase {
    private final CANSparkMax beltMotor = new CANSparkMax(CANConfig.INDEXER_BELT, MotorType.kBrushless);

    private final GenericEntry motorOutputEntry;

    public Indexer() {
        motorOutputEntry = Constants.INDEXER_TAB.add("Belt Motor Output", 0).getEntry();
    }

    private void outputToMotor(double percent) {
        beltMotor.set(percent);
        motorOutputEntry.setDouble(percent);
    }

    public Command spinBeltForward() {
        return Commands.runOnce(() -> outputToMotor(IndexerConfig.BELT_SPEED));
    }

    public Command spinBeltBackward() {
        return Commands.runOnce(() -> outputToMotor(-IndexerConfig.BELT_SPEED));
    }

    public Command stopBelt() {
        return Commands.runOnce(() -> outputToMotor(0));
    }
}
