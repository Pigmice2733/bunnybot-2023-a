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
    private final CANSparkMax overBumperMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(CANConfig.INTAKE_WHEELS, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(false);

        overBumperMotor = new CANSparkMax(CANConfig.OVER_BUMPER_INTAKE, MotorType.kBrushless);
        overBumperMotor.restoreFactoryDefaults();
        overBumperMotor.setInverted(false);

        ShuffleboardHelper.addOutput("Motor Output", Constants.INTAKE_TAB, () -> intakeMotor.get());
    }

    /** Sets the intake motor to a percent output (0.0 - 1.0) */
    public void outputToMotors(double percent) {
        intakeMotor.set(-percent);
        overBumperMotor.set(percent);
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
