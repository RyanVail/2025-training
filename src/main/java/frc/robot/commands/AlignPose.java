package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
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
    double endVelocity;

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
        this.endVelocity = 0.0;

        if (waypoints != null)
            setWaypoints(waypoints);
    }

    public void setWaypoints(List<Pose2d> waypoints) {
        waypoints.add(0, drive.getPose());
        this.waypoints = waypoints;
    }

    public void setEndVelocity(double vel) {
        this.endVelocity = vel;
    }

    public void initialize() {
        Pose2d start = waypoints.get(0);
        Pose2d end = waypoints.get(waypoints.size() - 1);

        // TODO: This is just for testing and should be disabled during comp.
        if (drive.getPose().getTranslation().getDistance(start.getTranslation()) >= Units.inchesToMeters(1)) {
            Commands.print(
                    "Starting position (" + start
                            + ") isn't current drive position (" + drive.getPose()
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

        trajectory = TrajectoryGenerator.generateTrajectory (
            waypoints,
            DriveConstants.TRAJECTORY_CONFIG.setStartVelocity(0) // TODO: Set this to a real value.
                .setEndVelocity(endVelocity)
        );

        Logger.recordOutput("AligningTo", end);

        if (trajectory == null) {
            Commands.print("Trajectory is null").schedule();
            cancel();
        }
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

        if (trajectory == null) {
            Commands.print("Trajectory is null").schedule();
            return;
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