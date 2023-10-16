package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.GrabberConfig;

public class Grabber extends SubsystemBase {
    private final CANSparkMax rotationMotor, flywheelsMotor;

    private final ProfiledPIDController rotationController;

    private double targetRotation, currentRotation;

    private final GenericEntry rotationOutputEntry, flywheelsOutputEntry;

    public Grabber() {
        rotationMotor = new CANSparkMax(CANConfig.GRABBER_ROTATION, MotorType.kBrushless);
        flywheelsMotor = new CANSparkMax(CANConfig.GRABBER_FLYWHEELS, MotorType.kBrushless);

        rotationMotor.restoreFactoryDefaults();
        flywheelsMotor.restoreFactoryDefaults();

        // Convert to arm rotations in degrees
        rotationMotor.getEncoder().setPositionConversionFactor(GrabberConfig.ROTATION_CONVERSION * 360);

        rotationController = new ProfiledPIDController(
                GrabberConfig.ARM_P, GrabberConfig.ARM_I, GrabberConfig.ARM_D,
                new Constraints(GrabberConfig.MAX_VELOCITY, GrabberConfig.MAX_ACCELERATION));

        ShuffleboardHelper.addOutput("Current", Constants.GRABBER_TAB, () -> currentRotation)
                .asDial(-180, 180);
        ShuffleboardHelper.addOutput("Setpoint", Constants.GRABBER_TAB, () -> rotationController.getSetpoint())
                .asDial(-180, 180);
        ShuffleboardHelper.addOutput("Target", Constants.GRABBER_TAB, () -> targetRotation)
                .asDial(-180, 180);

        rotationOutputEntry = Constants.GRABBER_TAB.add("Rotation Output", 0).getEntry();
        flywheelsOutputEntry = Constants.GRABBER_TAB.add("FLywheels Output", 0).getEntry();
    }

    @Override
    public void periodic() {
        currentRotation = rotationMotor.getEncoder().getPosition();
        updateClosedLoopControl();
    }

    /** Calculates and applys the next output from the PID controller */
    private void updateClosedLoopControl() {
        double calculatedOutput = rotationController.calculate(getCurrentRotation(), targetRotation);
        outputToRotationMotor(calculatedOutput);
    }

    public void outputToRotationMotor(double output) {
        rotationMotor.set(output);
        rotationOutputEntry.setDouble(output);
    }

    public void outputToFlywheelsMotor(double output) {
        flywheelsMotor.set(output);
        flywheelsOutputEntry.setDouble(output);
    }

    public void setTargetRotation(double targetRotation) {
        this.targetRotation = targetRotation;
    }

    public double getTargetRotation() {
        return targetRotation;
    }

    public double getCurrentRotation() {
        return currentRotation;
    }

    public Command runFlywheelsCommand(double percent) {
        return Commands.runOnce(() -> outputToFlywheelsMotor(percent));
    }

    public Command stopFlywheelsCommand() {
        return Commands.runOnce(() -> outputToFlywheelsMotor(0));
    }

    public Command setTargetArmAngleCommand() {
        return Commands.runOnce(() -> setTargetRotation(targetRotation));
    }

    public enum ArmPosition {
        Up,
        Middle,
        Down
    }

    public static double rotationFromState(ArmPosition state) {
        switch (state) {
            case Up:
                return GrabberConfig.ARM_UP_ANGLE;
            case Middle:
                return GrabberConfig.ARM_MID_ANGLE;
            case Down:
                return GrabberConfig.ARM_DOWN_ANGLE;
            default:
                return 0;
        }
    }
}
