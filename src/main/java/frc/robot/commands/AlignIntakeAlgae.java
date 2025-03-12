package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class AlignIntakeAlgae extends AlignPose {
    public AlignIntakeAlgae(Drive drive) {
        super(drive, null, AlignCamera.Front);
        super.setName("AlignIntakeAlgae");
    }

    @Override
    public void initialize() {
        Pose2d pose = (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
                ? FlippingUtil.flipFieldPose(drive.getPose())
                : drive.getPose();

        int index = getClosestPoseIndex(pose);
        Pose2d preintake_pose = FieldConstants.ALGAE_PREINTAKE_POSES[index];
        Pose2d intake_pose = FieldConstants.ALGAE_INTAKE_POSES[index];
        Commands.print("preintake_pose " + preintake_pose).schedule();
        Commands.print("intake_pose  " + intake_pose).schedule();

        // TODO: Prescore too.
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
            preintake_pose = FlippingUtil.flipFieldPose(preintake_pose);
            intake_pose = FlippingUtil.flipFieldPose(intake_pose);
        }

        Rotation2d angle = new Rotation2d(Units.degreesToRadians(60) * index);

        Commands.print("robot pose: " + drive.getPose()).schedule();
        Commands.print("reef tag pos: " + FieldConstants.REEF_TAG_POSITIONS[index]).schedule();

        Commands.print(
                "Align Offset: " +
                FieldConstants.REEF_TAG_POSITIONS[index].minus(
                        FlippingUtil.flipFieldPose(drive.getPose()).getTranslation()).rotateBy(angle)).schedule();

        setWaypoints(new ArrayList<Pose2d>(List.of(preintake_pose, intake_pose)));
        super.initialize();
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
