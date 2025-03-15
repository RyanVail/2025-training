package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionManager;

public class AlignFeed extends AlignPose {
    public AlignFeed(Drive drive) {
        super(drive, null, AlignCamera.Back);
        super.setName("AlignFeed");
    }

    @Override
    public void initialize() {
        super.setCameras();
        VisionManager.resetToCameraPose();
        Pose2d pose = drive.getPose();

        Logger.recordOutput("_EstimatedPose", pose);

        pose = (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
                ? FlippingUtil.flipFieldPose(pose)
                : pose;

        Pose2d target = pose.nearest(List.of(FieldConstants.FEEDER_POSES));

        // TODO: Make consisten with the others.
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
            target = FlippingUtil.flipFieldPose(target);
        }

        setTarget(target);
        super.initialize();
    }
}
