package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class AlignScoreCoral extends AlignPose {
    boolean left;

    public AlignScoreCoral(Drive drive, boolean left) {
        super(drive, null, AlignCamera.Front);
        this.left = left;
    }

    @Override
    public void initialize() {
        Pose2d pose = (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
                ? FlippingUtil.flipFieldPose(drive.getPose())
                : drive.getPose();

        int index = getCoralScoreIndex(pose);

        // Rotation2d angle = new Rotation2d(Units.degreesToRadians(-120))
        //         .rotateBy(new Rotation2d(Units.degreesToRadians(-60) * index));

        // Logger.recordOutput("Drive pose", drive.getPose());
        // Logger.recordOutput("Drive pose Flipped", FlippingUtil.flipFieldPose(drive.getPose()));

        // Logger.recordOutput(
        //         "Align Offset",
        //         FieldConstants.REEF_TAG_POSITIONS[index / 2].minus(
        //                 FlippingUtil.flipFieldPose(drive.getPose()).getTranslation()).rotateBy(angle));

        Pose2d align_pose = FieldConstants.CORAL_SCORE_POSES[index];
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            align_pose = FlippingUtil.flipFieldPose(align_pose);

        setWaypoints(List.of(align_pose));
        super.initialize();
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
        for (int i = 0; i < FieldConstants.ALGAE_INTAKE_POSES.length; i++) {
            Pose2d p = FieldConstants.ALGAE_INTAKE_POSES[i];
            double dist = p.getTranslation().getDistance(robot_pos);
            if (closest_dist >= dist) {
                closest_dist = dist;
                closest_pose = i;
            }
        }

        return (closest_pose << 1) | (left ? 0 : 1);
    }
}
