package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private VisionSubsystemIO io;
    private PhotonCamera frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);
    private PhotonCamera backCamera = new PhotonCamera(VisionConstants.BACK_CAMERA_NAME);
    private Supplier<Pose2d> poseSupplier;

    public VisionSubsystem(VisionSubsystemIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.poseSupplier = poseSupplier;
        // TODO: Add back and front
        io.init(frontCamera, backCamera);
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic(poseSupplier.get());
    }
}
