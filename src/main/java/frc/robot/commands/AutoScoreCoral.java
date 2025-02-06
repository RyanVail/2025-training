package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoScoreCoral extends Command {
    DriveSubsystem drive;
    ElevatorSubsystem elevator;
    VisionSubsystem vision;
    boolean left;

    /***
     * Auto scores coral. The robot must be aligned with the algae reef segment.
     */
    public AutoScoreCoral(
            DriveSubsystem drive,
            ElevatorSubsystem elevator,
            VisionSubsystem vision,
            boolean left) {
        super.addRequirements(drive, elevator, vision);
        this.drive = drive;
        this.elevator = elevator;
        this.vision = vision;
        this.left = left;
    }

    @Override
    public void initialize() {
        int index = getCoralScoreIndex(drive.getPose());
        int level = vision.getScorableLevel(index);
        double height = FieldConstants.CORAL_LEVEL_HEIGHTS[level] + ElevatorConstants.CORAL_OFFSET;

        new AlignPose(drive, FieldConstants.CORAL_SCORE_POSES[index])
            .alongWith(new ElevatorSetHeight(elevator, height))
            .schedule();
    }

    /**
     * Gets the index of the reef coral segment to score on either the left or right
     * side of the robot at the suppplied pose.
     * 
     * @return The index of the coral reef segment to score on.
     */
    private int getCoralScoreIndex(Pose2d pose) {
        Translation2d robot_pos = pose.getTranslation();
        double closest_dist = Double.MAX_VALUE;
        int closest_pose = 0;

        // The closest algae position is found to determine which face of the reef's
        // hexagon the robot is aligned to so that a left and right button can be used
        // to align with the coral segment of the reef.
        for (int i = 0; i < FieldConstants.ALGAE_REEF_POSES.length; i++) {
            Pose2d p = FieldConstants.ALGAE_REEF_POSES[i];
            double dist = p.getTranslation().getDistance(robot_pos);
            if (closest_dist >= dist) {
                closest_dist = dist;
                closest_pose = i;
            }
        }

        return (closest_pose << 1) | (left ? 0 : 1);
    }
}
