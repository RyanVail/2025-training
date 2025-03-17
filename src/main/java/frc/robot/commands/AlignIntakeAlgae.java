package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class AlignIntakeAlgae extends AlignPose {
    public AlignIntakeAlgae(Drive drive) {
        super(drive, null, AlignCamera.Front);
        super.setName("AlignIntakeAlgae");
    }

    private Pose2d intake_pose;
    private Pose2d starting_pose;

    @Override
    public void initialize() {
        super.setTarget(this.intake_pose);
        super.initialize();
    }

    /**
     * Finds and caches the target algae intake pose.
     * 
     * @warning This must be called before initializing this command.
     */
    public void findTargetPose() {
        this.starting_pose = drive.getPose();

        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            this.starting_pose = FlippingUtil.flipFieldPose(this.starting_pose);

        int index = getClosestPoseIndex(this.starting_pose);

        this.intake_pose = FieldConstants.ALGAE_INTAKE_POSES[index];
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            this.intake_pose = FlippingUtil.flipFieldPose(this.intake_pose);
    }

    /**
     * @return True if the alignment can run, false otherwise.
     * 
     * @warning findTargetPose must be called before this method.
     */
    public boolean canRun() {
        return super.withinStartingDistance(this.starting_pose, this.intake_pose);
    }

    @Override
    public boolean withinStartingDistance(Pose2d start, Pose2d target) {
        // canRun tests the distance.
        return true;
    }

    public int getClosestPoseIndex(Pose2d pose) {
        Translation2d robot_pos = pose.getTranslation();
        int closest_pose_index = 0;
        double closest_dist = Double.MAX_VALUE;

        for (int i = 0; i < FieldConstants.ALGAE_INTAKE_POSES.length; i++) {
            Pose2d p = FieldConstants.ALGAE_INTAKE_POSES[i];
            double dist = p.getTranslation().getDistance(robot_pos);
            if (closest_dist >= dist) {
                closest_pose_index = i;
                closest_dist = dist;
            }
        }

        return closest_pose_index;
    }
}
