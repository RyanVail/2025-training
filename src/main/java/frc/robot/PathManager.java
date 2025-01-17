package frc.robot;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PathManager {
    private static SendableChooser<PathPlannerPath> chooser = new SendableChooser<>();
    private static Command pathingCommand;

    private static final String[] paths = {
        "Example Path"
    };

    public static void configurePaths()
    {
        // try {
        //     for (String str : paths)
        //         chooser.addOption(str, PathPlannerPath.fromPathFile(str));
        // } catch (Exception e) {
        //     e.printStackTrace();
        // }

        // SmartDashboard.putData("Path", chooser);
        // SmartDashboard.putData("Start path", Commands.runOnce(() -> {
        //     runPath();
        // }));

        // SmartDashboard.putData("Pathfind and start path", Commands.runOnce(() -> {
        //     pathFindRunPath();
        // }));
    }

    private static void runPath()
    {
        // PathPlannerPath path = chooser.getSelected();
        // if (path != null)
        //     AutoBuilder.followPath(path).schedule();
    }

    private static void pathFindRunPath()
    {
        // PathPlannerPath path = chooser.getSelected();
        // if (path != null) {
        //     pathingCommand = AutoBuilder.pathfindThenFollowPath(path, Constants.DriveConstants.pathConstraints);
        //     pathingCommand.schedule();
        // }
    }

    public static void cancelPath()
    {
        if (pathingCommand != null) {
            pathingCommand.cancel();
        }
    }
}
