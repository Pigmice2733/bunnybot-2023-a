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
import frc.robot.Constants.GrabberConfig.ArmPosition;

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

    /** Sets the rotation motor percent output */
    public void outputToRotationMotor(double output) {
        rotationMotor.set(output);
        rotationOutputEntry.setDouble(output);
    }

    /** Sets the flywheels motor percent output */
    public void outputToFlywheelsMotor(double output) {
        flywheelsMotor.set(output);
        flywheelsOutputEntry.setDouble(output);
    }

    /** Sets the angle that the arm will go to */
    public void setTargetRotation(double targetRotation) {
        this.targetRotation = targetRotation;
    }

    /** @return the arms target rotation */
    public double getTargetRotation() {
        return targetRotation;
    }

    /** @return the arms current rotation */
    public double getCurrentRotation() {
        return currentRotation;
    }

    /** Sets the flywheels to intake bunnies */
    public Command runFlywheelsIntakeCommand(double percent) {
        return Commands.runOnce(() -> outputToFlywheelsMotor(GrabberConfig.FLYWHEEL_INTAKE_SPEED));
    }

    /** Sets the flywheels to eject bunnies */
    public Command runFlywheelsEjectCommand(double percent) {
        return Commands.runOnce(() -> outputToFlywheelsMotor(GrabberConfig.FLYWHEEL_EJECT_SPEED));
    }

    /** Sets the flywheels to zero output */
    public Command stopFlywheelsCommand() {
        return Commands.runOnce(() -> outputToFlywheelsMotor(0));
    }

    /** Sends the arm to the specified position */
    public Command setTargetArmAngleCommand(ArmPosition position) {
        return Commands.runOnce(() -> setTargetRotation(position.getAngle()));
    }
}
