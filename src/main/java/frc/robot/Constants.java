package frc.robot;

import org.dyn4j.geometry.Transform;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

                public static final double GEAR_RATIO = 1.0; // Direct drive
                public static final double MOMENT_OF_INERTIA = 0.002; // From CAD, in units of kilogram * meters squared

                public static final int PORT = 13;

                public static final int FIRST_SENSOR_ID = 0;
                public static final int END_SENSOR_ID = 1;

                public static final double ALGAE_SCORE_VEL = 100.0;
                public static final double ALGAE_SCORE_TIME = 1.0;

                public static final double CORAL_SCORE_VOLTAGE = -4.5;
                public static final double CORAL_FEED_VOLTAGE = 4.5;

                public static final double SENSE_TIME = 0.115;
        }

        public class DriveConstants {
                public static final double MAX_SPEED = Units.feetToMeters(15);
                public static final double DEADZONE = 0.08;

                public static final PathConstraints pathConstraints = new PathConstraints(
                                2.0,
                                2.0,
                                Units.degreesToRadians(540),
                                Units.degreesToRadians(720));

                public static final HolonomicDriveController driveController = new HolonomicDriveController(
                                new PIDController(10, 0.0, 0),
                                new PIDController(10, 0.0, 0),
                                new ProfiledPIDController(12.0, 0.0, 0.0,
                                                new TrapezoidProfile.Constraints(0.0, 0.0)));

                static {
                        driveController.setTolerance(new Pose2d(0.05, 0.05, new Rotation2d(0.05)));
                }

                public static final PPHolonomicDriveController PPDriveController = new PPHolonomicDriveController(
                                new PIDConstants(2.0, 0.0, 0.0),
                                new PIDConstants(2.0, 0.0, 0.0));

                public static final double ALIGN_DIST = Units.feetToMeters(0.1);

                public static final double ELEVATOR_HEIGHT_DIMMER = 0.1;

                public static final double SLEW_RATE = 5;
        }

        public class ElevatorConstants {
                public static final double GEAR_RATIO = 20.0;

                public static final double GEAR_PERIMETER = (1 + (11 / 16)) * Math.PI;
                public static final double METER_PER_ENCODER_UNIT = GEAR_PERIMETER / (GEAR_RATIO);

                public static final double MASS = Units.lbsToKilograms(50.0);
                public static final double RADIUS = Units.inchesToMeters(2.5);

                // TODO: Set to real values.
                public static final double MIN_HEIGHT = Units.inchesToMeters(4);
                public static final double MAX_HEIGHT = Units.inchesToMeters(72.0);

                public static final double HEIGHT = MAX_HEIGHT - MIN_HEIGHT;

                public static final Translation2d POS = new Translation2d(0.35, 0.0);

                public static final double P = 65.0;
                public static final double I = 0.0;
                public static final double D = 0.0;

                public static final double S = 0.4825;
                public static final double G = 0.0;
                public static final double V = 0.0;

                public static final double CORAL_SCORE_OFFSET = 0.0;

                public static final double CORAL_INTAKE_HEIGHT = 0.046;

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
                public static final Pose3d BACK_CAMERA_POSE = new Pose3d();
                public static final int BACK_CAMERA_WIDTH = 640;
                public static final int BACK_CAMERA_HEIGHT = 640;
                public static final Rotation2d BACK_CAMERA_FOV = Rotation2d.fromDegrees(90.0);
                public static final int BACK_CAMERA_FPS = 15;
                public static final double BACK_CAMERA_AVG_LATENCY_MS = 50.0;
                public static final double BACK_CAMERA_LATENCY_STD_DEV_MS = 15.0;
                public static final Transform3d BACK_CAMERA_TRANSFORM = new Transform3d(
                                0.0,
                                0.0,
                                0.0,
                                new Rotation3d(0, 0, Units.degreesToRadians(0)));

                public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout
                                .loadField(AprilTagFields.k2025ReefscapeWelded);
        }

        // TODO: Real values here.
        public class FieldConstants {
                public static final Pose2d[] CORAL_SCORE_POSES = {
                                new Pose2d(),
                                new Pose2d(),

                                // new Pose2d(4.09, 5.2, new Rotation2d(Units.degreesToRadians(-65))),
                                // new Pose2d(3.71, 5.02, new Rotation2d(Units.degreesToRadians(-65))),

                                new Pose2d(),
                                new Pose2d(),

                                new Pose2d(),
                                new Pose2d(),

                                new Pose2d(3.65, 3.01, new Rotation2d(Units.degreesToRadians(60))),
                                new Pose2d(3.94, 2.79, new Rotation2d(Units.degreesToRadians(60))),

                                new Pose2d(),
                                new Pose2d(),

                                new Pose2d(),
                                new Pose2d(),

                                // new Pose2d(4.92, 5.13, new Rotation2d(Units.degreesToRadians(-120))),
                                // new Pose2d(5.19, 4.97, new Rotation2d(Units.degreesToRadians(-120))),
                };

                public static final Pose2d[] ALGAE_REEF_POSES = {
                                // new Pose2d(3.89, 5.12, new Rotation2d(Units.degreesToRadians(-65))),
                                new Pose2d(),
                                new Pose2d(),
                                new Pose2d(),
                                new Pose2d(3.79, 2.93, new Rotation2d(Units.degreesToRadians(60))),
                                new Pose2d(),
                                new Pose2d(),
                                // new Pose2d(5.06, 5.05, new Rotation2d(Units.degreesToRadians(-120))),
                };

                public static final double[] CORAL_LEVEL_HEIGHTS = {
                                Units.inchesToMeters(1),
                                Units.inchesToMeters(4.7),
                                Units.inchesToMeters(9.5),
                                Units.inchesToMeters(20), // 20.25
                };

                public static final Pose2d[] FEEDER_POSES = {
                                new Pose2d(1.65, 0.75, new Rotation2d(Units.degreesToRadians(54))),
                                new Pose2d(0.63, 1.46, new Rotation2d(Units.degreesToRadians(54))),
                                new Pose2d(0.63, 6.66, new Rotation2d(Units.degreesToRadians(-54))),
                                new Pose2d(1.65, 7.33, new Rotation2d(Units.degreesToRadians(-54))),
                };

                public static final Translation2d[] FEEDER_POSITIONS = {
                                new Translation2d(1.14, 1.1),
                                new Translation2d(1.14, 7),
                };
        }

        public class EndEffectorConstants {
                public static final double P = 0.05;
                public static final double I = 0.0;
                public static final double D = 0.0;

                public static final double LENGTH = Units.inchesToMeters(12);

                public static final int PORT = 11;

                public static final Translation2d POS = new Translation2d(0.0, 0.0);

                public static final double GEAR_RATIO = 36.0;

                public static final double MOMENT_OF_INERTIA = 0.5;

                public static final double MIN_ANGLE = Units.degreesToRadians(-40);
                public static final double MAX_ANGLE = Units.degreesToRadians(240);

                public static final double STARTING_ANGLE = Units.degreesToRadians(0);
                public static final double VISUALIZATION_BASE_ANGLE = -90;

                // TODO: This is very wrong and will remain wrong till the PID is fixed.
                public static final double ALIGN_ANGLE = 15.0;

                public static final double PRESCORING_ANGLE = 145;
                public static final double[] SCORING_ANGLES = { 36.8, 75, 120 };

                public static final double INTAKE_ANGLE = 212;

                public static final double REQUIRED_ELEVATOR_HEIGHT = 0.045;
                public static final double MIN_ELEVATOR_REQUIRED_ANGLE = 45;
        }

        public class BeaterBarConstants {
                public static final int PORT = 14;

                public static final double FEED_SPEED = 0.1;
        }
}
