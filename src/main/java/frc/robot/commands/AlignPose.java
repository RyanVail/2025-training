package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionManager;

public class AlignPose extends Command {
    Drive drive;
    Pose2d target;
    AlignCamera camera;
    double endVelocity;
    boolean finished;

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

        SmartDashboard.putData("AlignXPID", DriveConstants.DRIVE_CONTROLLER.getXController());
        SmartDashboard.putData("AlignYPID", DriveConstants.DRIVE_CONTROLLER.getYController());
        SmartDashboard.putData("AlignThetaPID", DriveConstants.DRIVE_CONTROLLER.getThetaController());
    }

    public void setTarget(Pose2d target) {
        this.target = target;
    }

    public void initialize() {
        Pose2d start = drive.getPose();

        ChassisSpeeds speeds = drive.getRobotVelocity();
        DriveConstants.AUTO_ALIGN_X_SLEW.reset(speeds.vxMetersPerSecond);
        DriveConstants.AUTO_ALIGN_Y_SLEW.reset(speeds.vyMetersPerSecond);

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

        // Drive controllers have no builtin way of doing this because they're only
        // meant to follow a single trajectory.
        DriveConstants.DRIVE_CONTROLLER.getThetaController().reset(start.getRotation().getRadians());
        DriveConstants.DRIVE_CONTROLLER.getXController().reset();
        DriveConstants.DRIVE_CONTROLLER.getYController().reset();
    }

    public void setCameras() {
        if (camera == AlignCamera.Back) {
            VisionManager.onlyBack();
        } else if (camera == AlignCamera.Front) {
            VisionManager.onlyFront();
        } else if (camera == AlignCamera.All) {
            VisionManager.allCameras();
        } else {
            VisionManager.noCameras();
        }
    }

    @Override
    public void execute() {
        setCameras();

        // TOOD: Put the rotation somewhere.
        ChassisSpeeds speeds = DriveConstants.DRIVE_CONTROLLER.calculate(
                drive.getPose(),
                target,
                0.0,
                target.getRotation());

        speeds.vxMetersPerSecond = DriveConstants.AUTO_ALIGN_X_SLEW.calculate(speeds.vxMetersPerSecond);
        speeds.vyMetersPerSecond = DriveConstants.AUTO_ALIGN_Y_SLEW.calculate(speeds.vyMetersPerSecond);

        drive.driveRobotRelative(speeds);
    }

    @Override
    public boolean isFinished() {
        this.finished = DriveConstants.DRIVE_CONTROLLER.atReference();
        return DriveConstants.DRIVE_CONTROLLER.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        VisionManager.defaultCameras();
    }
}