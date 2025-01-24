package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private VisionSubsystemIO io;
    private PhotonCamera frontCamera = new PhotonCamera("front");

    public VisionSubsystem(VisionSubsystemIO io) {
        this.io = io;
        io.init(frontCamera);
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    @Override
    public void periodic() {
        // TODO: Can this be the real pose?
        Pose2d pose = new Pose2d();
        io.periodic(pose);
    }
}
