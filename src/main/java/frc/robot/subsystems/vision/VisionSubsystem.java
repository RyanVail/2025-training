package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionSubsystem {
    private VisionSubsystemIO io;

    public VisionSubsystem(VisionSubsystemIO io) {
        this.io = io;
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    public void simulationPeriodic(Pose2d pose) {
        io.simulationPeriodic(pose);
    }
}
