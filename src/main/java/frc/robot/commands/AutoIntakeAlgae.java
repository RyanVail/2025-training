package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class AutoIntakeAlgae extends AlignPose {
    public AutoIntakeAlgae(Drive drive) {
        super(drive, new Pose2d(), AlignCamera.Front);
    }

    @Override
    public void initialize() {
        Pose2d pose = (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
                ? FlippingUtil.flipFieldPose(drive.getPose())
                : drive.getPose();

        Pose2d align_pose = (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
                ? FlippingUtil.flipFieldPose(getClosestPose(pose))
                : getClosestPose(pose);

        Rotation2d angle = new Rotation2d(Units.degreesToRadians(-120))
                .rotateBy(new Rotation2d(Units.degreesToRadians(-60) * 2));

        Logger.recordOutput(
                "Align Offset",
                FieldConstants.REEF_TAG_POSITIONS[2].minus(
                        FlippingUtil.flipFieldPose(drive.getPose()).getTranslation()).rotateBy(angle));

        super.target_pose = align_pose;
        super.initialize();
    }

    public Pose2d getClosestPose(Pose2d pose) {
        Translation2d robot_pos = pose.getTranslation();
        Pose2d closest_pose = new Pose2d();
        double closest_dist = Double.MAX_VALUE;

        for (Pose2d p : FieldConstants.ALGAE_INTAKE_POSES) {
            double dist = p.getTranslation().getDistance(robot_pos);
            if (closest_dist >= dist) {
                closest_dist = dist;
                closest_pose = p;
            }
        }

        return closest_pose;
    }
}
