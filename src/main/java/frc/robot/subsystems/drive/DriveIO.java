package frc.robot.subsystems.drive;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface DriveIO {
    public void drive(ChassisSpeeds speeds);

    public SwerveModuleState[] getSwerveStates();

    public Rotation2d getRotation();

    public Pose2d getPose();

    public void resetPose(Pose2d pose);

    public ChassisSpeeds getCurrentSpeeds();

    public ChassisSpeeds getRobotRelativeSpeeds();

    public void addVisionEstimations(EstimatedRobotPose[] poses);

    public void periodic();
}