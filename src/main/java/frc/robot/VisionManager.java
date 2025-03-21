package frc.robot;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;

public class VisionManager {
    private static Drive drive;

    private static PhotonCamera frontCamera;
    private static PhotonCamera backCamera;

    private static PhotonPoseEstimator frontEstimator;
    private static PhotonPoseEstimator backEstimator;

    private static EstimatedRobotPose frontPose;
    private static EstimatedRobotPose backPose;

    private static boolean disableFront = false;
    private static boolean disableBack = true;

    public static void initialize(Drive drive) {
        VisionManager.drive = drive;

        frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);
        backCamera = new PhotonCamera(VisionConstants.BACK_CAMERA_NAME);

        try {
            frontEstimator = new PhotonPoseEstimator(
                    FieldConstants.LAYOUT,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    VisionConstants.FRONT_CAMERA_TRANSFORM);

            backEstimator = new PhotonPoseEstimator(
                    FieldConstants.LAYOUT,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    VisionConstants.BACK_CAMERA_TRANSFORM);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void resetToCameraPose() {
        if (!disableFront && frontPose != null
        && Units.millisecondsToSeconds(System.currentTimeMillis()) - frontPose.timestampSeconds < VisionConstants.MAX_SECONDS) {
            VisionManager.drive.resetPose(frontPose.estimatedPose.toPose2d());
        } else if (!disableBack && backPose != null
        && Units.millisecondsToSeconds(System.currentTimeMillis()) - backPose.timestampSeconds < VisionConstants.MAX_SECONDS) {
            VisionManager.drive.resetPose(backPose.estimatedPose.toPose2d());
        }
    }

    public static void onlyBack() {
        disableFront = true;
        disableBack = false;
    }

    public static void onlyFront() {
        disableFront = false;
        disableBack = true;
    }

    public static void allCameras() {
        disableFront = false;
        disableBack = false;
    }

    public static void noCameras() {
        disableFront = true;
        disableBack = true;
    }

    public static void defaultCameras() {
        if (DriverStation.isAutonomous()) {
            noCameras();
        } else {
            onlyFront();
        }
    }

    public static EstimatedRobotPose[] getEstimatedPoses() {
        var fp = frontEstimator.update(frontCamera.getLatestResult()).orElse(null);
        var bp = backEstimator.update(backCamera.getLatestResult()).orElse(null);

        if (fp != null)
            frontPose = fp;

        if (bp != null)
            backPose = bp;

        return new EstimatedRobotPose[] {
                disableFront ? null : fp,
                disableBack ? null : bp,
        };
    }
}
