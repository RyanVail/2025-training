package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystemIOSim implements VisionSubsystemIO {
    private VisionSystemSim visionSim;
    private PhotonCameraSim backCameraSim;
    private SimCameraProperties cameraProp;

    public VisionSubsystemIOSim() {
        visionSim = new VisionSystemSim("main");

        try {
            AprilTagFieldLayout tagLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
            visionSim.addAprilTags(tagLayout);
        } catch (Exception e) {
            e.printStackTrace();
        }

        cameraProp = new SimCameraProperties();
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setCalibration(VisionConstants.BACK_CAMERA_WIDTH, VisionConstants.BACK_CAMERA_HEIGHT,
                VisionConstants.BACK_CAMERA_FOV);
        cameraProp.setFPS(VisionConstants.BACK_CAMERA_FPS);
        cameraProp.setAvgLatencyMs(VisionConstants.BACK_CAMERA_AVG_LATENCY_MS);
        cameraProp.setLatencyStdDevMs(VisionConstants.BACK_CAMERA_LATENCY_STD_DEV_MS);
    }

    @Override
    public void init(PhotonCamera backCam) {
        backCameraSim = new PhotonCameraSim(backCam, cameraProp);
        backCameraSim.enableDrawWireframe(true);

        visionSim.addCamera(backCameraSim, VisionConstants.BACK_CAMERA_TRANSFORM);
    }

    @Override
    public void resetPose(Pose2d pose) {
        visionSim.resetRobotPose(pose);
    }

    @Override
    public void simulationPeriodic(Pose2d pose) {
        visionSim.update(pose);
        SmartDashboard.putData("VisionField", visionSim.getDebugField());
    }
}
