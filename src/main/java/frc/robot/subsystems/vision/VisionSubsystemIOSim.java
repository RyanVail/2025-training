package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import javax.xml.transform.TransformerConfigurationException;

import org.dyn4j.geometry.Vector3;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystemIOSim implements VisionSubsystemIO {
    private VisionSystemSim visionSim;
    private PhotonCameraSim backCameraSim;
    private SimCameraProperties cameraProp;

    public VisionSubsystemIOSim() {
        visionSim = new VisionSystemSim("main");

        try {
            AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
            visionSim.addAprilTags(tagLayout);
        } catch (Exception e) {
            e.printStackTrace();
        }

        cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);
    }

    @Override
    public void init(PhotonCamera backCam)
    {
        backCameraSim = new PhotonCameraSim(backCam, cameraProp);
        backCameraSim.enableDrawWireframe(true);

        // TODO: Make this a constant.
        visionSim.addCamera(backCameraSim, new Transform3d());
    }

    @Override
    public void resetPose(Pose2d pose) {
        visionSim.resetRobotPose(pose);
    }

    @Override
    public void periodic(Pose2d pose) {
        visionSim.update(pose);
        SmartDashboard.putData("VisionField", visionSim.getDebugField());
    }
}
