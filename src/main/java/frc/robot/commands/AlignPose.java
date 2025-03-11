package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionManager;

public class AlignPose extends Command {
    List<Pose2d> waypoints;

    Drive drive;
    Trajectory trajectory;
    AlignCamera camera;

    public enum AlignCamera {
        None,
        All,
        Back,
        Front,
    };

    public AlignPose(Drive drive, List<Pose2d> waypoints, AlignCamera camera) {
        addRequirements(drive);
        this.drive = drive;
        this.camera = camera;

        if (waypoints != null)
            setWaypoints(waypoints);
    }

    public void setWaypoints(List<Pose2d> waypoints) {
        waypoints.add(0, drive.getPose());
        this.waypoints = waypoints;
    }

    public void initialize() {
        Pose2d start = waypoints.get(0);
        Pose2d end = waypoints.get(waypoints.size() - 1);

        // TODO: This is just for testing and should be disabled during comp.
        if (!start.equals(drive.getPose())) {
            Commands.print(
                    "Starting position (" + drive.getPose()
                            + ") isn't current drive position (" + start
                            + ").")
                    .schedule();
        }

        double dist = start.getTranslation().getDistance(end.getTranslation());

        // If generateTrajectory is called with two identical translations it will
        // throw.
        if (dist <= DriveConstants.AUTO_ALIGN_CANCEL_DIST) {
            super.cancel();
            return;
        }

        // Ensuring the target position is within an acceptable distance.
        if (dist >= DriveConstants.AUTO_ALIGN_MAX_DIST) {
            super.cancel();
            return;
        }

        trajectory = TrajectoryGenerator.generateTrajectory(waypoints, DriveConstants.TRAJECTORY_CONFIG);
    }

    @Override
    public void execute() {
        if (camera == AlignCamera.Back) {
            VisionManager.onlyBack();
        } else if (camera == AlignCamera.Front) {
            VisionManager.onlyFront();
        } else if (camera == AlignCamera.All) {
            VisionManager.allCameras();
        } else {
            VisionManager.noCameras();
        }

        State state = trajectory.sample(System.currentTimeMillis() * 0.001);
        drive.driveRobotRelative(
                DriveConstants.DRIVE_CONTROLLER.calculate(drive.getPose(), state, state.poseMeters.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return DriveConstants.DRIVE_CONTROLLER.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        VisionManager.defaultCameras();
    }
}