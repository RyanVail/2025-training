package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionSubsystemIO {
    public void resetPose(Pose2d pose);

    public void periodic(Pose2d pose);

    public void init(PhotonCamera backCam);
}