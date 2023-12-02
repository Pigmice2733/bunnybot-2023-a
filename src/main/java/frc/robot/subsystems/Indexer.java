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
    // Controls the belt and mecanum wheels

    // Controls the wheel that feeds balls into the shooter
    private final CANSparkMax feederWheelMotor;

    public Indexer() {
        feederWheelMotor = new CANSparkMax(CANConfig.FEEDER_WHEEL, MotorType.kBrushless);

        feederWheelMotor.restoreFactoryDefaults();

        ShuffleboardHelper.addOutput("Feeder Output", Constants.INDEXER_TAB, () -> feederWheelMotor.get());
    }

    private void outputToFeeder(double percent) {
        feederWheelMotor.set(percent);
    }

    /** Runs the feeder wheel forward. */
    public Command spinFeederForward() {
        return Commands.runOnce(() -> outputToFeeder(IndexerConfig.FEEDER_SPEED));
    }

    /** Runs the feeder wheel backward. */
    public Command spinFeederBackwards() {
        return Commands.runOnce(() -> outputToFeeder(IndexerConfig.BACKWARD_FEEDER_SPEED));
    }

    // /** Sets the feeder wheel to zero output. */
    // public Command stopFeeder() {
    // return Commands.runOnce(() -> outputToFeeder(0));
    // }
}
