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
    private final CANSparkMax indexerMotor;

    // Controls the wheel that feeds balls into the shooter
    private final CANSparkMax feederWheelMotor;

    public Indexer() {
        indexerMotor = new CANSparkMax(CANConfig.INDEXER, MotorType.kBrushless);
        feederWheelMotor = new CANSparkMax(CANConfig.FEEDER_WHEEL, MotorType.kBrushless);

        indexerMotor.restoreFactoryDefaults();
        feederWheelMotor.restoreFactoryDefaults();

        ShuffleboardHelper.addOutput("Indexer Output", Constants.INDEXER_TAB, () -> indexerMotor.get());
        ShuffleboardHelper.addOutput("Feeder Output", Constants.INDEXER_TAB, () -> feederWheelMotor.get());
    }

    private void outputToIndexer(double percent) {
        indexerMotor.set(percent);
    }

    private void outputToFeeder(double percent) {
        feederWheelMotor.set(percent);
    }

    /** Runs the indexer belt forward. */
    public Command spinBeltForward() {
        return Commands.runOnce(() -> outputToIndexer(IndexerConfig.BELT_SPEED));
    }

    /** Runs the indexer belt backward. */
    public Command spinBeltBackward() {
        return Commands.runOnce(() -> outputToIndexer(-IndexerConfig.BELT_SPEED));
    }

    /** Sets the indexer belt to zero output. */
    public Command stopBelt() {
        return Commands.runOnce(() -> outputToIndexer(0));
    }

    /** Runs the feeder wheel forward. */
    public Command spinFeederForward() {
        return Commands.runOnce(() -> outputToFeeder(IndexerConfig.FEEDER_SPEED));
    }

    /** Runs the feeder wheel backward. */
    public Command spinFeederBackward() {
        return Commands.runOnce(() -> outputToFeeder(-IndexerConfig.FEEDER_SPEED));
    }

    /** Sets the feeder wheel to zero output. */
    public Command stopFeeder() {
        return Commands.runOnce(() -> outputToFeeder(0));
    }
}
