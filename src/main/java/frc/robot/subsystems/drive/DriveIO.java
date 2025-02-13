package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface DriveIO {
    public void drive(ChassisSpeeds speeds);

    public SwerveModuleState[] getSwerveStates();

    public Rotation2d getRotation();

    public Pose2d getPose();

    public void resetPose(Pose2d pose);

    public ChassisSpeeds getCurrentSpeeds();

    public ChassisSpeeds getRobotRelativeSpeeds();

    public void addVisionMeasurement(
        Pose2d visionMeasurement,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs);

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds);

    public void periodic();
}