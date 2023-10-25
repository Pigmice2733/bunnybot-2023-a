package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.IntakeConfig;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final GenericEntry motorSpeedEntry;

    public Intake() {
        intakeMotor = new CANSparkMax(CANConfig.INTAKE_WHEELS, MotorType.kBrushless);
        motorSpeedEntry = Constants.INTAKE_TAB.add("Motor Output", 0).getEntry();
    }

    private void outputToMotor(double percent) {
        intakeMotor.set(percent);
        motorSpeedEntry.setDouble(percent);
    }

    public Command spinForward() {
        return Commands.runOnce(() -> outputToMotor(IntakeConfig.INTAKE_SPEED));
    }

    public Command spinBackward() {
        return Commands.runOnce(() -> outputToMotor(-IntakeConfig.INTAKE_SPEED));
    }

    public Command stopWheels() {
        return Commands.runOnce(() -> outputToMotor(0));
    }
}
