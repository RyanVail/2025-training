package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;

public class AutoManager {
    private static SendableChooser<PathPlannerAuto> chooser = new SendableChooser<>();
    private static PathPlannerAuto auto;

    private static Drive drive;
    private static Elevator elevator;
    private static EndEffector endEffector;

    private static final String[] autos = {
            "left Coral Auto",
            "Right Coral Auto",
    };

    public static void configureAutos(Drive drive, Elevator elevator, EndEffector endEffector) {
        AutoManager.drive = drive;
        AutoManager.elevator = elevator;
        AutoManager.endEffector = endEffector;

        try {
            for (String str : autos)
                chooser.addOption(str, new PathPlannerAuto(str));
        } catch (Exception e) {
            e.printStackTrace();
        }

        SmartDashboard.putData("Auto", chooser);
    }

    public static void start() {
        VisionManager.noCameras();

        AutoManager.auto = chooser.getSelected();
        if (AutoManager.auto != null)
            AutoManager.auto.schedule();

        // TODO: This    has to be cached somewhere.
        // drive.setGryoOffset(auto.getStartingPose().getRotation());
    }

    public static void cancel() {
        if (AutoManager.auto != null)
            AutoManager.auto.cancel();

        VisionManager.defaultCameras();

        // This is done to prevent the scenario that the elevator or end effector tries
        // to go down while over the reef.
        AutoManager.elevator.setSetpoint(AutoManager.elevator.getHeight());
        AutoManager.endEffector.setSetpoint(AutoManager.endEffector.getAngle());
    }
}
