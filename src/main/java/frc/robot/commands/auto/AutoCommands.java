package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.commands.ElevatorSetHeight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;

public class AutoCommands {
    public static Command scoreAlgae(ElevatorSubsystem elevator, Flywheel flywheel) {
        return flywheel.setVelocity(FlywheelConstants.ALGAE_SCORE_VEL)
                .andThen(new WaitCommand(FlywheelConstants.ALGAE_SCORE_TIME))
                .andThen(flywheel.setVelocity(0.0));
    }

    /***
     * Auto scores coral using the vision subsystem. The robot must be aligned with
     * the reef segment.
     */
    public static Command autoScoreCoral(DriveSubsystem drive, ElevatorSubsystem elevator, VisionSubsystem vision,
            Flywheel flywheel,
            boolean left) {
        int index = getCoralScoreIndex(drive.getPose(), left);
        int level = vision.getScorableLevel(index);
        return new ElevatorSetHeight(elevator,
                FieldConstants.CORAL_LEVEL_HEIGHTS[level] + ElevatorConstants.CORAL_OFFSET)
                .andThen(pathTo(drive, FieldConstants.CORAL_SCORE_POSES[index]));
        // TODO: This can't lower, if it is going to lower auto then it needs to move
        // away before doing that.
    }

    /**
     * @brief Gets the index of the reef coral segment to score on either the left
     *        or right side of the robot at the suppplied pose.
     * 
     * @param robotPose The current pose of the robot.
     * @param left      If scoring on the left or right side of the robot.
     * @return The index of the coral reef segment to score on.
     */
    private static int getCoralScoreIndex(Pose2d robotPose, boolean left) {
        double closest_dist = Double.MAX_VALUE;
        int closest_pose = 0;

        // The closest algae position is found to determine which face of the reef's
        // hexagon the robot is aligned to so that a left and right button can be used
        // to align with the coral segment of the reef.
        for (int i = 0; i < FieldConstants.ALGAE_REEF_POSES.length; i++) {
            Pose2d p = FieldConstants.ALGAE_REEF_POSES[i];
            double dist = p.getTranslation().getDistance(robotPose.getTranslation());
            if (closest_dist >= dist) {
                closest_dist = dist;
                closest_pose = i;
            }
        }

        return (closest_pose << 1) | (left ? 0 : 1);
    }

    /**
     * @brief Finds a path to a pose and creates a command to follow that path.
     * 
     * @param pose The pose to align to.
     * @return The command to follow the found path.
     */
    public static Command pathTo(DriveSubsystem drive, Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, DriveConstants.pathConstraints, 0.0);
    }
}