package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    public void resetPose(Pose2d pose);

    public default void simulationPeriodic(Pose2d pose) {};

    public void init(PhotonCamera frontCam, PhotonCamera backCam);
}