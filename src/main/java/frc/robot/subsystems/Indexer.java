package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.IndexerConfig;

public class Indexer extends SubsystemBase {
    private final CANSparkMax beltMotor = new CANSparkMax(CANConfig.INDEXER_BELT, MotorType.kBrushless);

    public Indexer() {
        ShuffleboardHelper.addOutput("Motor Output", Constants.INDEXER_TAB, () -> beltMotor.get());
    }

    private void outputToMotor(double percent) {
        beltMotor.set(percent);
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
