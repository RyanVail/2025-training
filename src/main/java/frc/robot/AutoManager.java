package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionManager;

public class AutoManager {
    private static SendableChooser<PathPlannerAuto> chooser = new SendableChooser<>();
    private static PathPlannerAuto auto;

    // TODO: The gyro has to be reset too.
    private static Drive drive;

    private static final String[] autos = {
            "left Coral Auto",
            "Right Coral Auto",
    };

    public static void configureAutos(Drive drive) {
        AutoManager.drive = drive;

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

        drive.setGryoOffset(auto.getStartingPose().getRotation());
    }

    public static void cancel() {
        if (AutoManager.auto != null)
            AutoManager.auto.cancel();
    }
}
