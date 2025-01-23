package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystemIOSim implements VisionSubsystemIO {
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public VisionSubsystemIOSim() {
        PhotonCamera camera = new PhotonCamera("Bottom");

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
    }

    public void resetPose(Pose2d pose) {
        visionSim.resetRobotPose(pose);
    }

    public void simulationPeriodic(Pose2d pose) {
        visionSim.update(pose);
    }
}
