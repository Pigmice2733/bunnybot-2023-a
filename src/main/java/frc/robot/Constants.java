// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.PathConstraints;
import com.pigmice.frc.lib.drivetrain.swerve.SwerveConfig;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final ShuffleboardTab HOOD_TAB = Shuffleboard.getTab("Hood");
    public static final ShuffleboardTab TURRET_TAB = Shuffleboard
            .getTab("Turret");
    public static final ShuffleboardTab SWERVE_TAB = Shuffleboard
            .getTab("Drivetrain");
    public static final ShuffleboardTab VISION_TAB = Shuffleboard
            .getTab("Vision");
    public static final ShuffleboardTab GRABBER_TAB = Shuffleboard
            .getTab("Grabber");
    public static final ShuffleboardTab SHOOTER_TAB = Shuffleboard
            .getTab("Shooter");
    public static final ShuffleboardTab INDEXER_TAB = Shuffleboard
            .getTab("Indexer");
    public static final ShuffleboardTab INTAKE_TAB = Shuffleboard
            .getTab("Intake");
    public static final ShuffleboardTab DRIVER_TAB = Shuffleboard
            .getTab("Driver");

    public static final double AXIS_THRESHOLD = 0.25;

    public static final class CANConfig {
        public static final int FRONT_LEFT_DRIVE = 11; // done
        public static final int FRONT_LEFT_STEER = 10;// done
        public static final int FRONT_RIGHT_DRIVE = 13;// done
        public static final int FRONT_RIGHT_STEER = 12;// done
        public static final int BACK_LEFT_DRIVE = 16;// done
        public static final int BACK_LEFT_STEER = 17;// done
        public static final int BACK_RIGHT_DRIVE = 14;// done
        public static final int BACK_RIGHT_STEER = 15;// done

        public static final int FRONT_LEFT_ABS_ENCODER = 20;// done
        public static final int FRONT_RIGHT_ABS_ENCODER = 24;// done
        public static final int BACK_LEFT_ABS_ENCODER = 22;// done
        public static final int BACK_RIGHT_ABS_ENCODER = 26;// done

        public static final int ROTATE_TURRET = 35; // done
        public static final int ROTATE_SHOOTER = 32;

        public static final int OVER_BUMPER_INTAKE = 31;// done

        public static final int GRABBER_ROTATION = 40; // done
        public static final int GRABBER_FLYWHEELS_A = 41; // done
        public static final int GRABBER_FLYWHEELS_B = 42; // done

        public static final int INDEXER = 50;
        public static final int FEEDER_WHEEL = 51; // done

        public static final int HOOD_ROTATION = 60;

        public static final int INTAKE_WHEELS = 30;
    }

    public static final class TurretConfig {
        public static final double MANUAL_ROTATION_MULTIPLIER = 20;

        public static final double ROTATION_MOTOR_CONVERSION = 1 / 342.4; // encoder
        // rot to deg

        public static final double ROTATION_P = 0.038;
        public static final double ROTATION_I = 0.00;
        public static final double ROTATION_D = 0.00;

        public static final Constraints DEFAULT_CONSTRAINTS = new Constraints(
                200, 300);
        public static final Constraints WANDER_CONSTRAINTS = new Constraints(50,
                100);

        public static final double MAX_ALLOWED_ROTATION = 120; // degrees
        public static final double WANDER_LIMIT = MAX_ALLOWED_ROTATION - 10; // degrees

        public static final double TARGET_YAW_TOLERANCE = 3.5;

        public static final double MAX_FIRE_VELOCITY = 1; // max vel a shot is
                                                          // allowed at (TODO)

        public static final double ZERO_TURRET_SPEED = 0.1;

        public static final int LIMIT_SWITCH_PORT = 1;
    }

    public static final class ShooterConfig {

        private static final double ROTATION_GEAR_RATIO = 1;
        public static final double ROTATION_MOTOR_CONVERSION = ROTATION_GEAR_RATIO
                * 360; // encoder
                       // rot to
                       // deg

        public static final double MAX_VELOCITY = 50; // deg/sec
        public static final double MAX_ACCELERATION = 50; // deg/sec/sec

        public static final double DEFAULT_OUTPUT = 1;
        public static final double IDLE_SPEED = 0;
    }

    /*
     * public static class ShooterSetPoint {
     * public final double distance;
     * public final double flywheelSpeed;
     * public final double height;
     * 
     * public ShooterSetPoint(int d, int f, int h) {
     * distance = d;
     * flywheelSpeed = f;
     * height = h;
     * }
     * 
     * public ShooterSetPoint(double d, double f, double h) {
     * distance = d;
     * flywheelSpeed = f;
     * height = h;
     * }
     */

    public static final class HoodConfig {
        public static final double MANUAL_ROTATION_MULTIPLIER = 3;
        public static final double MOTOR_ZERO_SPEED = 0.075;

        // Encoder rotations to degrees
        public static final double ROTATION_MOTOR_CONVERSION = 1 / 120.0;

        public static final double HOOD_P = 0.056;
        public static final double HOOD_I = 0.0;
        public static final double HOOD_D = 0.0;

        public static final double MAX_VELOCITY = 450; // deg/sec
        public static final double MAX_ACCELERATION = 450; // deg/sec/sec

        public static final int LIMIT_SWITCH_PORT = 1;
    }

    public static final class IntakeConfig {
        public static final double INTAKE_SPEED = 0.3;
        public static final double OVER_BUMPER_SPEED_MULTIPLIER = 1;
    }

    public static final class IndexerConfig {
        public static final double BELT_SPEED = 0.15;
        public static final double FEEDER_SPEED = 0.75;
        public static final double BACKWARD_FEEDER_SPEED = -0.25;
    }

    public static final class GrabberConfig {
        public static final double ARM_P = 0.06;
        public static final double ARM_I = 0.000;
        public static final double ARM_D = 0.000;

        public static final double MAX_VELOCITY = 375; // 1200; // deg/sec
        public static final double MAX_ACCELERATION = 700; // 1200; // deg/sec/sec

        public static final double ROTATION_CONVERSION = 0.00007004 * 90; // encoder
                                                                          // rotation
                                                                          // to
                                                                          // arm
                                                                          // rotations
                                                                          // (TODO)

        public static final double MOTOR_ZERO_SPEED = 0.2;
        public static final double MOTOR_ZERO_CURRENT_THRESHOLD = 0.3;

        public static final int LIMIT_SWITCH_PORT = 0;

        // All in degrees
        public static enum ArmPosition {
            STOW, STORE, TOTE, GROUND, THROW
        }

        public static final double FLYWHEEL_INTAKE_SPEED = 0.6;
        public static final double FLYWHEEL_EJECT_SPEED = -0.4;
    }

    public static final class VisionConfig {
        public static final double TARGET_HEIGHT_METERS = Units
                .inchesToMeters(42);
        public static final double TURRET_HEIGHT_METERS = Units
                .inchesToMeters(37.25);

        public static final double CAMERA_HEIGHT_METERS = 1; // TODO
        public static final double CAMERA_PITCH_RADIANS = 0; // TODO
    }

    public final static class DrivetrainConfig {
        public static final double MAX_DRIVE_SPEED = 4.5; // max meters / second
        public static final double MAX_TURN_SPEED = 5; // max radians / second

        public static final double TRACK_WIDTH_METERS = 0.5842; // distance from
                                                                // the center of
                                                                // one
                                                                // wheel to
                                                                // another

        public static final double SLOWMODE_MULTIPLIER = 0.25; // TODO discuss
                                                               // with drive
                                                               // team

        private final static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(TRACK_WIDTH_METERS / 2,
                        TRACK_WIDTH_METERS / 2), // Front left
                new Translation2d(TRACK_WIDTH_METERS / 2,
                        -TRACK_WIDTH_METERS / 2), // Front right
                new Translation2d(-TRACK_WIDTH_METERS / 2,
                        TRACK_WIDTH_METERS / 2), // Back left
                new Translation2d(-TRACK_WIDTH_METERS / 2,
                        -TRACK_WIDTH_METERS / 2) // Back right
        );

        // Constants found in Sysid (volts)
        private static final SimpleMotorFeedforward DRIVE_FEED_FORWARD = new SimpleMotorFeedforward(
                0.35493, 2.3014, 0.12872);

        // From what I have seen, it is common to only use a P value in path following
        private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 2); // 3, 2.5
        private static final PIDController PATH_DRIVE_PID = new PIDController(0.3, 0, 0);
        private static final PIDController PATH_TURN_PID = new PIDController(0.31, 0, 0);

        // Offset from chassis center that the robot will rotate about
        private static final Translation2d ROTATION_CENTER_OFFSET = /*
                                                                     * new
                                                                     * Translation2d
                                                                     * (Units.
                                                                     * inchesToMeters
                                                                     * (-5),
                                                                     * Units.
                                                                     * inchesToMeters
                                                                     * (-10))
                                                                     */

                new Translation2d();

        private static final MkSwerveModuleBuilder FRONT_LEFT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(SWERVE_TAB
                        .getLayout("Front Left", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.FRONT_LEFT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.FRONT_LEFT_STEER)
                .withSteerEncoderPort(CANConfig.FRONT_LEFT_ABS_ENCODER)
                .withSteerOffset(Math.toRadians(73));

        private static final MkSwerveModuleBuilder FRONT_RIGHT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(SWERVE_TAB
                        .getLayout("Front Right", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(1, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.FRONT_RIGHT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.FRONT_RIGHT_STEER)
                .withSteerEncoderPort(CANConfig.FRONT_RIGHT_ABS_ENCODER)
                .withSteerOffset(Math.toRadians(-99));

        private static final MkSwerveModuleBuilder BACK_LEFT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(SWERVE_TAB
                        .getLayout("Back Left", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(2, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.BACK_LEFT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.BACK_LEFT_STEER)
                .withSteerEncoderPort(CANConfig.BACK_LEFT_ABS_ENCODER)
                .withSteerOffset(Math.toRadians(219));

        private static final MkSwerveModuleBuilder BACK_RIGHT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(SWERVE_TAB
                        .getLayout("Back Right", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(3, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.BACK_RIGHT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.BACK_RIGHT_STEER)
                .withSteerEncoderPort(CANConfig.BACK_RIGHT_ABS_ENCODER)
                .withSteerOffset(Math.toRadians(-285));

        public static final SwerveConfig SWERVE_CONFIG = new SwerveConfig(
                FRONT_LEFT_MODULE, FRONT_RIGHT_MODULE, BACK_LEFT_MODULE,
                BACK_RIGHT_MODULE,
                PATH_CONSTRAINTS, PATH_DRIVE_PID, PATH_TURN_PID,
                MAX_DRIVE_SPEED, MAX_TURN_SPEED,
                SLOWMODE_MULTIPLIER, KINEMATICS, DRIVE_FEED_FORWARD, SWERVE_TAB,
                ROTATION_CENTER_OFFSET);
    }

    public static class AutoConfig {
        // All times in seconds
        public static final double FEED_SHOOTER_INDEX_TIME = 0.15;
        public static final double TIME_BETWEEN_SHOTS = 1;
        public static final double SHOOTER_SPINUP_TIME = 1.2;
        public static final double EJECT_ALL_TIME = 5;

        // All motor outputs in percent (-1 -> 1)
        public static final double SHOOTER_EJECTION_SPEED = -0.1;

        public static final String FETCH_BUNNY_PATH = "fetch-bunny";
        public static final String FETCH_BUNNY_PATH_NO_SHOOT = "fetch-bunny Copy";

    }
}
