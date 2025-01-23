package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.PIDConstants;

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
        
        public static final double P = 2.0;
        public static final double I = 1.35;
        public static final double D = 0.75;

        public static final double S = 0.35;
        public static final double G = 2.0;
        public static final double V = 0.025;
    }
}
