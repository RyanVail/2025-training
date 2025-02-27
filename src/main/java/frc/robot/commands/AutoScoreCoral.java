package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class AutoScoreCoral extends Command {
    Drive drive;
    Elevator elevator;
    boolean left;

    /***
     * Auto scores coral. The robot must be aligned with the algae reef segment.
     */
    public AutoScoreCoral(
            Drive drive,
            Elevator elevator,
            boolean left) {
        super.addRequirements(drive, elevator);
        this.drive = drive;
        this.elevator = elevator;
        this.left = left;
    }

    @Override
    public void initialize() {
        int index = 0;
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
            index = getCoralScoreIndex(FlippingUtil.flipFieldPose(drive.getPose()));
        } else {
            index = getCoralScoreIndex(drive.getPose());
        }

        Logger.recordOutput("Drive pose", drive.getPose());
        Logger.recordOutput("Drive pose Flipped", FlippingUtil.flipFieldPose(drive.getPose()));

        Pose2d pose = FieldConstants.CORAL_SCORE_POSES[index];
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            pose = FlippingUtil.flipFieldPose(pose);

        Logger.recordOutput("_Target Pose", pose);
        new AlignPose(drive, pose).schedule();
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
