package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import frc.robot.Constants.VisionConstants;

public class VisionIOSim implements VisionIO {
    private VisionSystemSim visionSim;
    private PhotonCameraSim frontCameraSim;
    private PhotonCameraSim backCameraSim;
    private SimCameraProperties frontCameraProp;
    private SimCameraProperties backCameraProp;

    public VisionIOSim() {
        visionSim = new VisionSystemSim("main");

        try {
            // TODO: Choose the right layout.
            AprilTagFieldLayout tagLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
            visionSim.addAprilTags(tagLayout);
        } catch (Exception e) {
            e.printStackTrace();
        }

        backCameraProp = new SimCameraProperties();
        backCameraProp.setCalibError(0.35, 0.10);
        backCameraProp.setCalibration(VisionConstants.BACK_CAMERA_WIDTH, VisionConstants.BACK_CAMERA_HEIGHT,
                VisionConstants.BACK_CAMERA_FOV);
        backCameraProp.setFPS(VisionConstants.BACK_CAMERA_FPS);
        backCameraProp.setAvgLatencyMs(VisionConstants.BACK_CAMERA_AVG_LATENCY_MS);
        backCameraProp.setLatencyStdDevMs(VisionConstants.BACK_CAMERA_LATENCY_STD_DEV_MS);

        frontCameraProp = new SimCameraProperties();
        frontCameraProp.setCalibError(0.35, 0.10);
        frontCameraProp.setCalibration(VisionConstants.FRONT_CAMERA_WIDTH, VisionConstants.BACK_CAMERA_HEIGHT,
                VisionConstants.FRONT_CAMERA_FOV);
        frontCameraProp.setFPS(VisionConstants.FRONT_CAMERA_FPS);
        frontCameraProp.setAvgLatencyMs(VisionConstants.FRONT_CAMERA_AVG_LATENCY_MS);
        frontCameraProp.setLatencyStdDevMs(VisionConstants.FRONT_CAMERA_LATENCY_STD_DEV_MS);
    }

    @Override
    public void init(PhotonCamera frontCam, PhotonCamera backCam) {
        frontCameraSim = new PhotonCameraSim(frontCam, frontCameraProp);
        backCameraSim = new PhotonCameraSim(backCam, backCameraProp);

        frontCameraSim.enableDrawWireframe(true);
        backCameraSim.enableDrawWireframe(true);

        visionSim.addCamera(frontCameraSim, VisionConstants.FRONT_CAMERA_TRANSFORM);
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
