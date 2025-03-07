package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionManager;

public class AlignPose extends Command {
    protected Pose2d target_pose;
    Drive drive;
    Trajectory trajectory;
    AlignCamera camera;

    public enum AlignCamera {
        All,
        Back,
        Front,
    };

    public AlignPose(Drive drive, Pose2d target_pose, AlignCamera camera) {
        addRequirements(drive);

        this.drive = drive;
        this.target_pose = target_pose;

        // SmartDashboard.putData("AutoAlignXPID", DriveConstants.DRIVE_CONTROLLER.getXController());
        // SmartDashboard.putData("AutoAlignYPID", DriveConstants.DRIVE_CONTROLLER.getYController());
        // SmartDashboard.putData("AutoAlignAlignPID", DriveConstants.DRIVE_CONTROLLER.getThetaController());
    }

    public void initialize() {
        // TODO: This should check if it's on the position and cancel if so.
        trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(drive.getPose(), target_pose),
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
        } else {
            VisionManager.allCameras();
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
        VisionManager.allCameras();
    }
}