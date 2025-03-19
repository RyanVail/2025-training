package frc.robot.subsystems.drive;

import java.io.File;
import java.io.IOException;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class DriveIOSwerve implements DriveIO {
    private SwerveDrive swerveDrive;

    public DriveIOSwerve() {
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Units.feetToMeters(12.5));
        } catch (IOException e) {
            e.printStackTrace();
        }

        swerveDrive.setCosineCompensator(false);
        swerveDrive.setHeadingCorrection(false);

        swerveDrive.setOdometryPeriod(Constants.LOOP_TIME);

        // TODO: Is this required?
        // swerveDrive.setAutoCenteringModules(true);
        // for (SwerveModule mod : swerveDrive.getModules()) {
        //     mod.setAntiJitter(false);
        // }
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
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    @Override
    public void periodic() {
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
    public void addVisionEstimations(EstimatedRobotPose[] poses) {
        for (EstimatedRobotPose pose : poses)
            if (pose != null && pose.estimatedPose != null)
                swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                        pose.timestampSeconds);
    }

    @Override
    public Rotation2d getGyroRotation() {
        return swerveDrive.getGyroRotation3d().toRotation2d();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }
}