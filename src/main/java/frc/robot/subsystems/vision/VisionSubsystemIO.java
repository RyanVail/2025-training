package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionSubsystemIO {
    public void resetPose(Pose2d pose);

    public default void simulationPeriodic(Pose2d pose) {};
}