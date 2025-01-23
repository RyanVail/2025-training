package frc.robot.subsystems.drive;

import java.io.File;
import java.io.IOException;

import org.dyn4j.geometry.Rotation;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Commands;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class DriveSubsystemIOSwerve implements DriveSubsystemIO {
    private SwerveDrive swerveDrive;
    // SparkFlex moby = new SparkFlex(1, MotorType.kBrushless); // TODO: This is
    // tmp!

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

        // SparkFlexConfig config = new SparkFlexConfig();
        // config.apply(config.closedLoop.pid(0.002, 0.0, 0.0));
        // moby.configure(config, ResetMode.kNoResetSafeParameters,
        // PersistMode.kPersistParameters);
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
        // SwerveModuleState states[] = swerveDrive.kinematics
        // .toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(10, 0, 0,
        // Rotation2d.fromDegrees(0)));
        // swerveDrive.setModuleStates(states, false);
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