package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.GrabberConfig;
import frc.robot.Constants.GrabberConfig.ArmPosition;

public class Grabber extends SubsystemBase {
    private final CANSparkMax rotationMotor, flywheelsMotorA, flywheelsMotorB;

    private final DigitalInput limitSwitch;

    private final ProfiledPIDController rotationController;

    private boolean runPID = true;

    private double targetRotation;

    public Grabber() {
        rotationMotor = new CANSparkMax(CANConfig.GRABBER_ROTATION, MotorType.kBrushless);
        flywheelsMotorA = new CANSparkMax(CANConfig.GRABBER_FLYWHEELS_A, MotorType.kBrushless);
        flywheelsMotorB = new CANSparkMax(CANConfig.GRABBER_FLYWHEELS_B, MotorType.kBrushless);

        rotationMotor.restoreFactoryDefaults();
        flywheelsMotorA.restoreFactoryDefaults();
        flywheelsMotorB.restoreFactoryDefaults();

        rotationMotor.setInverted(false);
        flywheelsMotorA.setInverted(false);
        flywheelsMotorB.setInverted(true);

        rotationMotor.setIdleMode(IdleMode.kCoast);

        limitSwitch = new DigitalInput(GrabberConfig.LIMIT_SWITCH_PORT);

        // Convert to arm rotations in degrees
        rotationMotor.getEncoder()
                .setPositionConversionFactor(GrabberConfig.ROTATION_CONVERSION * 360);

        rotationController = new ProfiledPIDController(
                GrabberConfig.ARM_P, GrabberConfig.ARM_I, GrabberConfig.ARM_D,
                new Constraints(GrabberConfig.MAX_VELOCITY, GrabberConfig.MAX_ACCELERATION));

        setEncoderPosition(0);

        ShuffleboardHelper.addOutput("Current", Constants.GRABBER_TAB, () -> getCurrentRotation())
                .asDial(-180, 180);
        ShuffleboardHelper
                .addOutput("Setpoint", Constants.GRABBER_TAB,
                        () -> rotationController.getSetpoint().position)
                .asDial(-180, 180);
        ShuffleboardHelper.addOutput("Target", Constants.GRABBER_TAB, () -> targetRotation)
                .asDial(-180, 180);

        ShuffleboardHelper.addOutput("Rotation Output", Constants.GRABBER_TAB,
                () -> rotationMotor.get()).asDial(-1, 1);
        ShuffleboardHelper.addOutput("Flywheels Output", Constants.GRABBER_TAB,
                () -> flywheelsMotorA.get()).asDial(-1, 1);

        ShuffleboardHelper.addOutput("Limit Switch Pressed", Constants.GRABBER_TAB, () -> limitSwitchPressed());
        ShuffleboardHelper.addOutput("Flywheel vel", Constants.GRABBER_TAB,
                () -> rotationMotor.getEncoder().getVelocity());
    }

    @Override
    public void periodic() {
        updateClosedLoopControl();
    }

    /** Calculates and applies the next output from the PID controller. */
    private void updateClosedLoopControl() {
        if (!runPID)
            return;

        double calculatedOutput = rotationController.calculate(getCurrentRotation(),
                targetRotation);
        outputToRotationMotor(calculatedOutput);
    }

    /** Sets the rotation motor percent output. */
    public void outputToRotationMotor(double output) {
        rotationMotor.set(-output);
    }

    /** Sets the flywheels motor percent output. */
    public void outputToFlywheelsMotor(double output) {
        flywheelsMotorA.set(output);
        flywheelsMotorB.set(output);
    }

    /** Sets the angle that the arm will go to. */
    public void setTargetRotation(double targetRotation) {
        this.targetRotation = targetRotation;
    }

    /** Returns the arm's target rotation. */
    public double getTargetRotation() {
        return targetRotation;
    }

    /** Returns the arm's current rotation. */
    public double getCurrentRotation() {
        return -rotationMotor.getEncoder().getPosition();
    }

    /** Sets the flywheels to intake bunnies. */
    public Command runFlywheelsIntakeCommand() {
        return Commands.runOnce(() -> outputToFlywheelsMotor(GrabberConfig.FLYWHEEL_INTAKE_SPEED));
    }

    /** Sets the flywheels to eject bunnies. */
    public Command runFlywheelsEjectCommand() {
        return Commands.runOnce(() -> outputToFlywheelsMotor(GrabberConfig.FLYWHEEL_EJECT_SPEED));
    }

    /** Sets the flywheels to zero output. */
    public Command stopFlywheelsCommand() {
        return Commands.runOnce(() -> outputToFlywheelsMotor(0));
    }

    /** Sends the arm to the specified position. */
    public Command setTargetArmAngleCommand(ArmPosition position) {
        switch (position) {
            case STOW:
                return Commands.runOnce(() -> setTargetRotation(15));
            case STORE:
                return Commands.runOnce(() -> setTargetRotation(45));
            case TOTE:
                return Commands.runOnce(() -> setTargetRotation(147));
            case GROUND:
                return Commands.runOnce(() -> setTargetRotation(181));
            case THROW:
                return Commands.runOnce(() -> setTargetRotation(90));
            default:
                return Commands.none();
        }
    }

    public Command setControllerConstraints(double velocity, double acceleration, double p) {
        return Commands.runOnce(() -> {
            rotationController.setConstraints(new Constraints(velocity, acceleration));
            rotationController.setP(p);
        });
    }

    public double getRotationMotorCurrent() {
        return rotationMotor.getOutputCurrent();
    }

    public void setEncoderPosition(double position) {
        rotationMotor.getEncoder().setPosition(position);
    }

    public void stopPID() {
        runPID = false;
    }

    public void startPID() {
        runPID = true;
    }

    public void resetPID() {
        rotationController.reset(getCurrentRotation());
        setTargetRotation(getCurrentRotation());
    }

    public boolean limitSwitchPressed() {
        return !limitSwitch.get();
    }
}
