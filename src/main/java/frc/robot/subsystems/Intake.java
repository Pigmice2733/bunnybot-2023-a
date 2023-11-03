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
    private final CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(CANConfig.INTAKE_WHEELS, MotorType.kBrushless);
        ShuffleboardHelper.addOutput("Motor Output", Constants.INTAKE_TAB, () -> intakeMotor.get());
    }

    private void outputToMotor(double percent) {
        intakeMotor.set(percent);
    }

    /** Spins intake wheels to intake balls. */
    public Command spinForward() {
        return Commands.runOnce(() -> outputToMotor(IntakeConfig.INTAKE_SPEED));
    }

    /** Spins intake wheels to eject balls. */
    public Command spinBackward() {
        return Commands.runOnce(() -> outputToMotor(-IntakeConfig.INTAKE_SPEED));
    }

    /** Sets intake wheels to zero output. */
    public Command stopWheels() {
        return Commands.runOnce(() -> outputToMotor(0));
    }
}
