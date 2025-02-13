package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private VisionIO io;
    private PhotonCamera frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);
    private PhotonCamera backCamera = new PhotonCamera(VisionConstants.BACK_CAMERA_NAME);
    private Supplier<Pose2d> poseSupplier;

    // Algae on the reef.
    private boolean[][] algae = new boolean[2][6];

    // Coral on the reef.
    private boolean[][] coral = new boolean[3][12];

    public Vision(VisionIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.poseSupplier = poseSupplier;
        io.init(frontCamera, backCamera);
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    @Override
    public void periodic() {
        //
    }

    // TODO: Implement this or remove it.
    public Optional<Integer> getCoralLevel(PhotonTrackedTarget target) {
        Optional<Integer> result = Optional.empty();
        return result;
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic(poseSupplier.get());
    }

    public int getScorableLevel(int reef_id)
    {
        if (!coral[2][reef_id]) return 3;

        // Can't score coral on L1 or L2 when there's algae.
        if (algae[0][reef_id / 2] || algae[1][reef_id / 2])
            return 0;

        if (!coral[1][reef_id]) return 2;
        if (!coral[0][reef_id]) return 1;

        // No valid level was found.
        return 0;
    }
}
