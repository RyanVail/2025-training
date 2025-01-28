package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

        public static final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
                new PIDConstants(2.0, 0.0, 0.0),
                new PIDConstants(2.0, 0.0, 0.0));
    }

    public class ElevatorConstants {
        public static final double GEAR_RATIO = 1.0;
        public static final double MASS = Units.lbsToKilograms(2.0);
        public static final double RADIUS = Units.inchesToMeters(2.5);

        public static final double MIN_HEIGHT = Units.inchesToMeters(0.0);
        public static final double MAX_HEIGHT = Units.inchesToMeters(45.0);

        public static final double HEIGHT = MAX_HEIGHT - MIN_HEIGHT;

        public static final double POS_X = 0.0;
        public static final double POS_Y = 0.05;

        public static final double P = 12.0;
        public static final double I = 1.1;
        public static final double D = 2.0;

        public static final double S = 1.25;
        public static final double G = 0.5;
        public static final double V = 0.025;
    }

    public class VisionConstants {
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
                new Rotation3d());
    }
}
