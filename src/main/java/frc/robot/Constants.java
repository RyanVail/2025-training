package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class Constants {
        public static final int CONTROLLER_PORT = 0;

        public class IntakeConstants {
                public static final double P = 0.05;
                public static final double I = 0.0;
                public static final double D = 0.0;

                public static final double S = 0.0;
                public static final double V = 0.0;
                public static final double A = 0.0;

                public static final double GEAR_RATIO = 12.0;
                public static final double MOMENT_OF_INERTIA = 0.002;

                public static final int PORT = 13;

                public static final int FIRST_SENSOR_ID = 0;
                public static final int END_SENSOR_ID = 1;

                public static final double ALGAE_SCORE_VEL = 100.0;
                public static final double ALGAE_SCORE_TIME = 1.0;

                public static final double CORAL_SCORE_VOLTAGE = -15;
                public static final double CORAL_INTAKE_VOLTAGE = 15;
                public static final double CORAL_INTAKE_REV = 1.2;
                public static final double CORAL_EJECT_TIME = 0.2;

                public static final double ALGAE_SCORE_VOLTAGE = -6.0;

                public static final double ALGAE_EJECT_TIME = 0.5;
                public static final double ALGAE_STALL_VOLTAGE = 0.2;
                public static final double ALGAE_INTAKE_VOLTAGE = 4.0;

                // TODO: See if unstall is even required.
                public static final double ALGAE_INTAKE_START_VEL = 1700.0;
                public static final double ALGAE_INTAKE_STALL_VEL = 1600.0;
                public static final double ALGAE_INTAKE_UNSTALL_VEL = 800.0;
        }

        public class DriveConstants {
                public static final double MAX_SPEED = Units.feetToMeters(15);
                public static final double DEADZONE = 0.08;

                public static final double CORAL_SCORE_RESET_VEL = Units.feetToMeters(0.2);
                public static final double CORAL_SCORE_RESET_DIST = Units.inchesToMeters(3);

                public static final PathConstraints pathConstraints = new PathConstraints(
                                2.0,
                                2.0,
                                Units.degreesToRadians(540),
                                Units.degreesToRadians(720));

                public static final PPHolonomicDriveController PPDriveController = new PPHolonomicDriveController(
                                new PIDConstants(2.0, 0.0, 0.0),
                                new PIDConstants(2.0, 0.0, 0.0));

                public static final HolonomicDriveController DRIVE_CONTROLLER = new HolonomicDriveController(
                                new PIDController(5, 0.0, 0.0),
                                new PIDController(5, 0.0, 0.0),
                                new ProfiledPIDController(2.5, 0.0, 0.0,
                                                new TrapezoidProfile.Constraints(2.0, 0.5)));

                public static final TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(
                                2.0,
                                0.5);

                static {
                        DRIVE_CONTROLLER.setTolerance(new Pose2d(Units.inchesToMeters(0.3), Units.inchesToMeters(0.3),
                                        new Rotation2d(Units.degreesToRadians(0.5))));
                }

                public static final double MIN_ALIGN_DIST = Units.feetToMeters(0.02);
                public static final double MIN_ALIGN_ANGLE = Units.degreesToRadians(0.5);

                public static final double SLEW_RATE = 1;
                public static final double ROT_SLEW_RATE = 4;

                public static final double AUTO_ALIGN_MAX_DIST = Units.feetToMeters(7.5);

                public static final SlewRateLimiter[] X_SLEW_LIMITERS = {
                                new SlewRateLimiter(0.8),
                                new SlewRateLimiter(0.75),
                                new SlewRateLimiter(0.72),
                                new SlewRateLimiter(0.50),
                };

                public static final SlewRateLimiter[] Y_SLEW_LIMITERS = {
                                new SlewRateLimiter(0.8),
                                new SlewRateLimiter(0.75),
                                new SlewRateLimiter(0.72),
                                new SlewRateLimiter(0.50),
                };

                public static final SlewRateLimiter[] ROT_SLEW_LIMITERS = {
                                new SlewRateLimiter(2.5),
                                new SlewRateLimiter(2.125),
                                new SlewRateLimiter(1.25),
                                new SlewRateLimiter(1.0),
                };

                public static final double[] HEIGHT_LEVELS = {
                                Units.inchesToMeters(-1.0),
                                Units.inchesToMeters(3.0),
                                Units.inchesToMeters(8.0),
                                Units.inchesToMeters(15.0),
                };

                public static final double[] MAX_SPEEDS = {
                                1.0,
                                0.65,
                                0.23,
                                0.08,
                };

                public static final double AUTO_ALIGN_CANCEL_DIST = Units.inchesToMeters(0.025);
        }

        public class ElevatorConstants {
                public static final double GEAR_RATIO = 20.0;

                public static final double GEAR_PERIMETER = (1 + (11 / 16)) * Math.PI;
                public static final double METER_PER_ENCODER_UNIT = GEAR_PERIMETER / (GEAR_RATIO);

                public static final double MASS = Units.lbsToKilograms(40.0);
                public static final double RADIUS = Units.inchesToMeters(2.5);

                // TODO: Set to real values.
                public static final double MIN_HEIGHT = Units.inchesToMeters(4);
                public static final double MAX_HEIGHT = Units.inchesToMeters(72.0);

                public static final double HEIGHT = MAX_HEIGHT - MIN_HEIGHT;

                public static final Translation2d POS = new Translation2d(0.35, 0.0);

                public static final double P = 190;
                public static final double I = 0.0;
                public static final double D = 0.0;

                public static final double S = 0.325;
                public static final double G = 0.0;
                public static final double V = 0.0;

                public static final TrapezoidProfile PROFILE = new TrapezoidProfile(
                                new Constraints(Units.feetToMeters(5.0), Units.feetToMeters(3.25)));

                public static final double CORAL_SCORE_OFFSET = 0.0;

                public static final double CORAL_INTAKE_HEIGHT = 0.055;

                public static final double ALIGN_DIST_METERS = Units.inchesToMeters(0.4);

                public static final int LEFT_PORT = 8;
                public static final int RIGHT_PORT = 9;
        }

        public class VisionConstants {
                public static final String FRONT_CAMERA_NAME = "Front";
                public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(
                                new Translation3d(
                                                Units.inchesToMeters(10.728),
                                                Units.inchesToMeters(16.05),
                                                Units.inchesToMeters(9.361)),
                                new Rotation3d(0, 0, Units.degreesToRadians(-10)));
                public static final int FRONT_CAMERA_WIDTH = 640;
                public static final int FRONT_CAMERA_HEIGHT = 640;
                public static final Rotation2d FRONT_CAMERA_FOV = Rotation2d.fromDegrees(90.0);
                public static final int FRONT_CAMERA_FPS = 15;
                public static final double FRONT_CAMERA_AVG_LATENCY_MS = 50.0;
                public static final double FRONT_CAMERA_LATENCY_STD_DEV_MS = 15.0;

                public static final String BACK_CAMERA_NAME = "Back";
                public static final int BACK_CAMERA_WIDTH = 640;
                public static final int BACK_CAMERA_HEIGHT = 640;
                public static final Rotation2d BACK_CAMERA_FOV = Rotation2d.fromDegrees(90.0);
                public static final int BACK_CAMERA_FPS = 15;
                public static final double BACK_CAMERA_AVG_LATENCY_MS = 50.0;
                public static final double BACK_CAMERA_LATENCY_STD_DEV_MS = 15.0;
                public static final Transform3d BACK_CAMERA_TRANSFORM = new Transform3d(
                                Units.inchesToMeters(2.584),
                                Units.inchesToMeters(0.0),
                                Units.inchesToMeters(37.167),
                                // TODO: Finish this calibration.
                                new Rotation3d(
                                                0,
                                                Units.degreesToRadians(180 - 20),
                                                0));
        }

        public class FieldConstants {
                public static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout
                                .loadField(AprilTagFields.k2025ReefscapeWelded);

                public static final Translation2d[] REEF_TAG_POSITIONS = {
                                LAYOUT.getTagPose(20).orElse(new Pose3d()).toPose2d().getTranslation(),
                                LAYOUT.getTagPose(21).orElse(new Pose3d()).toPose2d().getTranslation(),
                                LAYOUT.getTagPose(22).orElse(new Pose3d()).toPose2d().getTranslation(),
                                LAYOUT.getTagPose(17).orElse(new Pose3d()).toPose2d().getTranslation(),
                                LAYOUT.getTagPose(18).orElse(new Pose3d()).toPose2d().getTranslation(),
                                LAYOUT.getTagPose(19).orElse(new Pose3d()).toPose2d().getTranslation(),
                };

                private static Translation2d LEFT_CORAL_SCORE_OFFSET = new Translation2d(0.056, 0.691);
                private static Translation2d RIGHT_CORAL_SCORE_OFFSET = new Translation2d(0.315, 0.553);
                private static Rotation2d BASE_CORAL_SCORE_ROTATION = new Rotation2d(Units.degreesToRadians(-120));

                public static final Pose2d[] CORAL_SCORE_POSES = new Pose2d[12];

                static {
                        Translation2d left_offset = LEFT_CORAL_SCORE_OFFSET;
                        Translation2d right_offset = RIGHT_CORAL_SCORE_OFFSET;
                        Rotation2d rot = BASE_CORAL_SCORE_ROTATION;
                        Rotation2d angle = new Rotation2d(Units.degreesToRadians(-60));

                        for (int i = 0; i < CORAL_SCORE_POSES.length >> 1; i++) {
                                CORAL_SCORE_POSES[(i << 1) | 0] = new Pose2d(REEF_TAG_POSITIONS[i].plus(left_offset),
                                                rot);
                                CORAL_SCORE_POSES[(i << 1) | 1] = new Pose2d(REEF_TAG_POSITIONS[i].plus(right_offset),
                                                rot);

                                left_offset = left_offset.rotateBy(angle);
                                right_offset = right_offset.rotateBy(angle);
                                rot = rot.rotateBy(angle);
                        }
                }

                private static Translation2d ALGAE_INTAKE_OFFSET = new Translation2d(-0.118, -0.585)
                                .rotateBy(new Rotation2d(Units.degreesToRadians(180)));

                // TODO: Find a real value for this.
                private static Translation2d ALGAE_PREINTAKE_OFFSET = new Translation2d(-0.26, -0.92)
                        .rotateBy(new Rotation2d(Units.degreesToRadians(180)));

                public static final Pose2d[] ALGAE_PREINTAKE_POSES = new Pose2d[6];
                public static final Pose2d[] ALGAE_INTAKE_POSES = new Pose2d[6];

                static {
                        Translation2d intake_offset = ALGAE_INTAKE_OFFSET;
                        Translation2d preintake_offset = ALGAE_PREINTAKE_OFFSET;
                        Rotation2d angle = new Rotation2d(Units.degreesToRadians(-60));
                        Rotation2d rot = BASE_CORAL_SCORE_ROTATION;

                        for (int i = 0; i < ALGAE_INTAKE_POSES.length; i++) {
                                ALGAE_INTAKE_POSES[i] = new Pose2d(REEF_TAG_POSITIONS[i].plus(intake_offset), rot);
                                ALGAE_PREINTAKE_POSES[i] = new Pose2d(REEF_TAG_POSITIONS[i].plus(preintake_offset),
                                                rot);

                                intake_offset = intake_offset.rotateBy(angle);
                                preintake_offset = preintake_offset.rotateBy(angle);
                                rot = rot.rotateBy(angle);
                        }
                }

                // TODO: These should be in the elevator constants.
                public static final double[] CORAL_LEVEL_HEIGHTS = {
                                Units.inchesToMeters(1),
                                Units.inchesToMeters(4.25),
                                Units.inchesToMeters(10.0),
                                Units.inchesToMeters(21.75),
                };

                public static final double PROCESSOR_HEIGHT = Units.inchesToMeters(2.1);

                public static final double[] ALGAE_LEVEL_HEIGHTS = {
                                Units.inchesToMeters(9),
                                Units.inchesToMeters(15),
                };

                public static final Pose2d[] FEEDER_POSES = {
                                new Pose2d(1.2, 1.234, new Rotation2d(1)),
                };
        }

        public class EndEffectorConstants {
                public static final double P = 0.2;
                public static final double I = 0.0;
                public static final double D = 0.0;

                public static final TrapezoidProfile PROFILE = new TrapezoidProfile(
                                new Constraints(650, 350));

                public static final double LENGTH = Units.inchesToMeters(12);

                public static final int PORT = 11;

                public static final Translation2d POS = new Translation2d(0.0, 0.0);

                public static final double GEAR_RATIO = 36.0;

                public static final double MOMENT_OF_INERTIA = 0.5;

                public static final double MIN_ANGLE = Units.degreesToRadians(-40);
                public static final double MAX_ANGLE = Units.degreesToRadians(240);

                public static final double IDLE_ANGLE = 15;

                public static final double STARTING_ANGLE = Units.degreesToRadians(0);
                public static final double VISUALIZATION_BASE_ANGLE = -90;

                public static final double ALIGN_ANGLE = 15.0;

                public static final double PRESCORING_ANGLE = 145;
                public static final double[] SCORING_ANGLES = { 50.7, 75, 82.6 };

                public static final double INTAKE_ANGLE = 208;
                public static final double ALGAE_INTAKE_ANGLE = 190;

                public static final double REQUIRED_ELEVATOR_HEIGHT = 0.037;
                public static final double MIN_ELEVATOR_REQUIRED_ANGLE = 55;

                public static final double PROCESSOR_ANGLE = 250;
        }

        public class BeaterBarConstants {
                public static final int PORT = 14;

                public static final double INTAKE_SPEED = 0.12;
        }
}
