package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.IntakeConfig;

public class Intake extends SubsystemBase {
    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;

    public Intake() {
        topMotor = new CANSparkMax(CANConfig.INTAKE_WHEELS, MotorType.kBrushless);
        bottomMotor = new CANSparkMax(CANConfig.INTAKE_WHEELS, MotorType.kBrushless);

        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        topMotor.setInverted(false);
        bottomMotor.setInverted(true);

        ShuffleboardHelper.addOutput("Top Output", Constants.INTAKE_TAB, () -> topMotor.get());
        ShuffleboardHelper.addOutput("Bottom Output", Constants.INTAKE_TAB, () -> bottomMotor.get());

    }

    private void outputToMotors(double percent) {
        topMotor.set(percent);
        bottomMotor.set(percent);
    }

    /** Spins intake wheels to intake balls. */
    public Command spinForward() {
        return Commands.runOnce(() -> outputToMotors(IntakeConfig.INTAKE_SPEED));
    }

    /** Spins intake wheels to eject balls. */
    public Command spinBackward() {
        return Commands.runOnce(() -> outputToMotors(-IntakeConfig.INTAKE_SPEED));
    }

    /** Sets intake wheels to zero output. */
    public Command stopWheels() {
        return Commands.runOnce(() -> outputToMotors(0));
    }
}
