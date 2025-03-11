package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class VisionManager {
    private static PhotonCamera frontCamera;
    private static PhotonCamera backCamera;

    private static PhotonPoseEstimator frontEstimator;
    private static PhotonPoseEstimator backEstimator;

    private static boolean disableFront = false;
    private static boolean disableBack = true;

    // // Algae on the reef.
    // private static boolean[][] algae = new boolean[2][6];

    // // Coral on the reef.
    // private static boolean[][] coral = new boolean[3][12];

    public static void initialize() {
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

    // public int getScorableLevel(int reef_id)
    // {
    // if (!coral[2][reef_id]) return 3;

    // // Can't score coral on L1 or L2 when there's algae.
    // if (algae[0][reef_id / 2] || algae[1][reef_id / 2])
    // return 0;

    // if (!coral[1][reef_id]) return 2;
    // if (!coral[0][reef_id]) return 1;

    // // No valid level was found.
    // return 0;
    // }

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
        onlyFront();
    }

    public static EstimatedRobotPose[] getEstimatedPoses() {
        return new EstimatedRobotPose[] {
                disableFront ? null : frontEstimator.update(frontCamera.getLatestResult()).orElse(null),
                disableBack ? null : backEstimator.update(backCamera.getLatestResult()).orElse(null),
        };
    }
}
