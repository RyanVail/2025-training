package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoManager {
    private static SendableChooser<PathPlannerAuto> chooser = new SendableChooser<>();
    private static Command command;

    private static final String[] autos = {
            "S0 Greedy"
    };

    public static void configureAutos() {
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
