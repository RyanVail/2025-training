package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final int CONTROLLER_PORT = 0;

    public class FlywheelConstants {
        public static final double P = 0.0125;
        public static final double I = 0.025;
        public static final double D = 0.0;

        // TODO: This doesn't go to zero.
        public static final double S = 1.1;
        public static final double V = 0.0;
        public static final double A = 0.0;

        public static final double GEAR_RATIO = 1.0; // Direct drive
        public static final double MOMENT_OF_INERTIA = 0.002; // From CAD, in units of kilogram * meters squared

        public static final int FLYWHEEL_PORT = 9;

        public static final double ALGAE_SCORE_VEL = 100.0;
        public static final double ALGAE_SCORE_TIME = 1.0;

        public static final double CORAL_SCORE_VEL = 10.0;
        public static final double CORAL_SCORE_TIME = 1.0;
    }

    public class ArmConstants {
        public static final double P = 5.25;
        public static final double I = 0.025;
        public static final double D = 0.0;

        public static final double LENGTH = Units.inchesToMeters(29.240);
        public static final double MASS = Units.lbsToKilograms(21.853);

        public static final double GEAR_RATIO = 200.0;

        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = (3.0 * Math.PI) / 4.0;
    }

    public class DriveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(12);
        public static final double DEADZONE = 0.08;

        public static final PathConstraints pathConstraints = new PathConstraints(
                2.0,
                2.0,
                Units.degreesToRadians(540),
                Units.degreesToRadians(720));

        public static final HolonomicDriveController driveController = new HolonomicDriveController(
                new PIDController(2.0, 0.0, 0.0),
                new PIDController(2.0, 0.0, 0.0),
                new ProfiledPIDController(2.0, 0.0, 0.0,
                        new TrapezoidProfile.Constraints(0.0, 0.0)));

        static { driveController.setTolerance(new Pose2d(0.05, 0.05, new Rotation2d(0.05))); }

        public static final PPHolonomicDriveController PPDriveController = new PPHolonomicDriveController(
                new PIDConstants(2.0, 0.0, 0.0),
                new PIDConstants(2.0, 0.0, 0.0));

        public static final double ALIGN_DIST = Units.feetToMeters(0.1);
    }

    public class ElevatorConstants {
        public static final double GEAR_RATIO = 1.0;
        public static final double MASS = Units.lbsToKilograms(2.0);
        public static final double RADIUS = Units.inchesToMeters(2.5);

        public static final double MIN_HEIGHT = Units.inchesToMeters(8.0);
        public static final double MAX_HEIGHT = Units.inchesToMeters(72.0);

        public static final double HEIGHT = MAX_HEIGHT - MIN_HEIGHT;

        public static final double POS_X = 0.0;
        public static final double POS_Y = 0.05;

        public static final double P = 12.0;
        public static final double I = 1.1;
        public static final double D = 2.0;

        public static final double S = 1.25;
        public static final double G = 0.5;
        public static final double V = 0.025;

        public static final double CORAL_OFFSET = 0.0;

        public static final double ELEVATOR_ALIGN_DIST = 0.15;
    }

    public class VisionConstants {
        public static final String FRONT_CAMERA_NAME = "Front";
        public static final Pose3d FRONT_CAMERA_POSE = new Pose3d();
        public static final int FRONT_CAMERA_WIDTH = 640;
        public static final int FRONT_CAMERA_HEIGHT = 640;
        public static final Rotation2d FRONT_CAMERA_FOV = Rotation2d.fromDegrees(90.0);
        public static final int FRONT_CAMERA_FPS = 15;
        public static final double FRONT_CAMERA_AVG_LATENCY_MS = 50.0;
        public static final double FRONT_CAMERA_LATENCY_STD_DEV_MS = 15.0;
        public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(
                0.0,
                0.0,
                0.0,
                new Rotation3d());

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
                new Rotation3d(0, 0, Units.degreesToRadians(90)));
    }

    // TODO: Real values here.
    public class FieldConstants {
        public static final Pose2d[] CORAL_SCORE_POSES = {
                new Pose2d(4.09, 5.2, new Rotation2d(Units.degreesToRadians(-65))),
                new Pose2d(3.71, 5.02, new Rotation2d(Units.degreesToRadians(-65))),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                new Pose2d()
        };

        public static final Pose2d[] ALGAE_REEF_POSES = {
                new Pose2d(3.89, 5.12, new Rotation2d(Units.degreesToRadians(-65))),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
                new Pose2d(),
        };

        public static final double[] CORAL_LEVEL_HEIGHTS = {
                Units.inchesToMeters(12),
                Units.inchesToMeters(30),
                Units.inchesToMeters(40),
                Units.inchesToMeters(65)
        };
    }
}
