package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AutoManager {
    private static SendableChooser<PathPlannerAuto> chooser = new SendableChooser<>();
    private static Command command;

    // TODO: Reset pose after-.
    private static Drive drive;

    private static final String[] autos = {
            "S0 Greedy",
            "Test Auto",
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
        command = chooser.getSelected();
        if (command != null)
            command.schedule();
    }

    public static void cancel() {
        if (command != null)
            command.cancel();
    }
}
