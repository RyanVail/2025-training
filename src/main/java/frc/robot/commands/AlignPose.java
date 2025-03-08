package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionManager;

public class AlignPose extends Command {
    protected Pose2d target_pose;
    Drive drive;
    Trajectory trajectory;
    AlignCamera camera;

    public enum AlignCamera {
        None,
        All,
        Back,
        Front,
    };

    public AlignPose(Drive drive, Pose2d target_pose, AlignCamera camera) {
        addRequirements(drive);

        this.drive = drive;
        this.target_pose = target_pose;

        SmartDashboard.putData("AutoAlignXPID", DriveConstants.DRIVE_CONTROLLER.getXController());
        SmartDashboard.putData("AutoAlignYPID", DriveConstants.DRIVE_CONTROLLER.getYController());
        SmartDashboard.putData("AutoAlignAlignPID", DriveConstants.DRIVE_CONTROLLER.getThetaController());
    }

    public void initialize() {
        Pose2d robot_pose = drive.getPose();

        double dist = robot_pose.getTranslation().getDistance(target_pose.getTranslation());

        // If generateTrajectory is called with two identical translations it will throw.
        if (dist <= DriveConstants.AUTO_ALIGN_CANCEL_DIST) {
            super.cancel();
            return;
        }

        // Ensuring the target position is within an acceptable distance.
        if (dist >= DriveConstants.AUTO_ALIGN_MAX_DIST) {
            super.cancel();
            return;
        }

        trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(robot_pose, target_pose),
                DriveConstants.TRAJECTORY_CONFIG);
    }

    @Override
    public void execute() {
        Pose2d drive_pose = drive.getPose();
        if (target_pose.getTranslation().getDistance(drive_pose.getTranslation()) >= DriveConstants.AUTO_ALIGN_MAX_DIST) {
            Logger.recordOutput("_CancelingAlignTo", target_pose);
            super.cancel();
            return;
        }

        if (camera == AlignCamera.Back) {
            VisionManager.onlyBack();
        } else if (camera == AlignCamera.Front) {
            VisionManager.onlyFront();
        } else if (camera == AlignCamera.All) {
            VisionManager.allCameras();
        } else {
            VisionManager.noCameras();
        }

        drive.driveRobotRelative(DriveConstants.DRIVE_CONTROLLER.calculate(
                drive.getPose(),
                trajectory.sample(System.currentTimeMillis() * 0.001),
                target_pose.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return DriveConstants.DRIVE_CONTROLLER.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        VisionManager.onlyFront(); // TODO: There should be a normal one.
    }
}