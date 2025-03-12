package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionManager;

public class AlignPose extends Command {
    Drive drive;
    Pose2d target;
    AlignCamera camera;
    double endVelocity;

    public enum AlignCamera {
        None,
        All,
        Back,
        Front,
    };

    public AlignPose(Drive drive, Pose2d target, AlignCamera camera) {
        addRequirements(drive);
        this.target = target;
        this.drive = drive;
        this.camera = camera;
        this.endVelocity = 0.0;
    }

    public void setTarget(Pose2d target) {
        this.target = target;
    }

    public void initialize() {
        Pose2d start = drive.getPose();

        // TODO: This is just for testing and should be disabled during comp.
        if (drive.getPose().getTranslation().getDistance(start.getTranslation()) >= Units.inchesToMeters(1)) {
            Commands.print(
                    "Starting position (" + start
                            + ") isn't current drive position (" + drive.getPose()
                            + ").")
                    .schedule();
        }

        double dist = start.getTranslation().getDistance(target.getTranslation());

        // If generateTrajectory is called with two identical translations it will
        // throw.
        if (dist <= DriveConstants.AUTO_ALIGN_CANCEL_DIST) {
            Commands.print("Auto align dist too close").schedule();
            super.cancel();
            return;
        }

        // Ensuring the target position is within an acceptable distance.
        if (dist >= DriveConstants.AUTO_ALIGN_MAX_DIST) {
            Commands.print("Auto align dist too far").schedule();
            super.cancel();
            return;
        }

        Logger.recordOutput("AligningTo", target);
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

        // TOOD: Put the rotation somewhere.
        drive.driveRobotRelative(DriveConstants.DRIVE_CONTROLLER.calculate(drive.getPose(), target, 0.0, target.getRotation()));
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