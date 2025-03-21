package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.VisionManager;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class AlignFeed extends AlignPose {
    public AlignFeed(Drive drive) {
        super(drive, null, AlignCamera.BACK);
        super.setName("AlignFeed");
    }

    @Override
    public void initialize() {
        super.setCameras();
        VisionManager.resetToCameraPose();
        Pose2d pose = drive.getPose();

        pose = (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
                ? FlippingUtil.flipFieldPose(pose)
                : pose;

        Pose2d target = pose.nearest(List.of(FieldConstants.FEEDER_POSES));

        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            target = FlippingUtil.flipFieldPose(target);

        super.setTarget(new Target(target, AutoAlignConstants.FEEDER_ALIGN_CONSTRAINTS));
        super.initialize();
    }
}
