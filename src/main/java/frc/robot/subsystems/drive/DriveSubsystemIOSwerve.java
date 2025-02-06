package frc.robot.subsystems.drive;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;

public class DriveSubsystemIOSwerve implements DriveSubsystemIO {
    private SwerveDrive swerveDrive;

    public DriveSubsystemIOSwerve() {
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Units.feetToMeters(12.5));
        } catch (IOException e) {
            e.printStackTrace();
        }

        swerveDrive.setCosineCompensator(false);
        swerveDrive.setHeadingCorrection(false);

        for (SwerveModule mod : swerveDrive.getModules()) {
            mod.setAntiJitter(false);
        }
    }

    @Override
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    @Override
    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    @Override
    public ChassisSpeeds getCurrentSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getFieldVelocity();
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
    }

    @Override
    public Rotation2d getRotation() {
        return swerveDrive.getPose().getRotation();
    }

    @Override
    public SwerveModuleState[] getSwerveStates() {
        return swerveDrive.getStates();
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionMeasurement,
            double timestampSeconds,
            Matrix<N3, N1> stdDevs) {
        swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
                visionMeasurement,
                timestampSeconds,
                stdDevs);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }
}