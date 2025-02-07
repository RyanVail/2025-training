package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoFeedCoral extends Command {
    DriveSubsystem drive;
    boolean left;

    public AutoFeedCoral(DriveSubsystem drive, boolean left) {
        super.addRequirements(drive);
        this.drive = drive;
        this.left = left;
    }

    @Override
    public void initialize() {
        int index = getFeederIndex(drive.getPose());

        // TODO: This is dumb.
        new AlignPose(drive, FieldConstants.FEEDER_POSES[index]).schedule();
    }

    private int getFeederIndex(Pose2d pose) {
        Translation2d robot_pos = pose.getTranslation();
        double closest_dist = Double.MAX_VALUE;
        int closest_pose = 0;

        for (int i = 0; i < FieldConstants.FEEDER_POSITIONS.length; i++) {
            double dist = FieldConstants.FEEDER_POSITIONS[i].getDistance(robot_pos);
            if (closest_dist >= dist) {
                closest_dist = dist;
                closest_pose = i;
            }
        }

        return (closest_pose << 1) | (left ? 0 : 1);
    }
}
