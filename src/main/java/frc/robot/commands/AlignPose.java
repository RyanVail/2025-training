package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.control.BetterTrapezoidProfile.State;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionManager;

public class AlignPose extends Command {
    Drive drive;
    Pose2d target;
    AlignCamera camera;

    State lastXState;
    State lastYState;

    State XStateSetpoint;
    State YStateSetpoint;

    boolean finished;

    public enum AlignCamera {
        None,
        All,
        Back,
        Front,
    };

    // TODO: Add and use this.
    // public class Target {
    //     // The target pose.
    //     Pose2d pose;

    //     // The minimum required distance from the target.
    //     double dist;

    //     // The minimum required distance from the rotation.
    //     double rot_dist;

    //     // The target velocity at the pose.
    //     double vel;
    // };

    public AlignPose(Drive drive, Pose2d target, AlignCamera camera) {
        addRequirements(drive);
        this.target = target;
        this.drive = drive;
        this.camera = camera;

        if (this.target != null)
            setTarget(this.target);

        SmartDashboard.putData("AlignXPID", DriveConstants.AUTO_ALIGN_X_CONTROLLER);
        SmartDashboard.putData("AlignYPID", DriveConstants.AUTO_ALIGN_Y_CONTROLLER);
        SmartDashboard.putData("AlignThetaPID", DriveConstants.AUTO_ALIGN_THETA_CONTROLLER);
    }

    public void setTarget(Pose2d target) {
        this.target = target;

        Pose2d start = drive.getPose();

        ChassisSpeeds speeds = drive.getRobotVelocity();

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

        XStateSetpoint = new State(target.getX(), 0);
        YStateSetpoint = new State(target.getY(), 0);

        lastXState = new State(start.getX(), speeds.vxMetersPerSecond);
        lastYState = new State(start.getY(), speeds.vyMetersPerSecond);

        DriveConstants.AUTO_ALIGN_X_CONTROLLER.reset();
        DriveConstants.AUTO_ALIGN_Y_CONTROLLER.reset();
        DriveConstants.AUTO_ALIGN_THETA_CONTROLLER.reset();

        DriveConstants.AUTO_ALIGN_THETA_CONTROLLER.setSetpoint(target.getRotation().getRadians());
    }

    public void initialize() {
        if (finished && this.target != null)
            this.setTarget(this.target);

        this.finished = false;
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
        if (this.target == null)
            return;

        setCameras();

        Pose2d pose = drive.getPose();

        lastXState = DriveConstants.AUTO_ALIGN_X_PROFILE.calculate(0.02, XStateSetpoint, lastXState);
        lastYState = DriveConstants.AUTO_ALIGN_Y_PROFILE.calculate(0.02, YStateSetpoint, lastYState);

        DriveConstants.AUTO_ALIGN_X_CONTROLLER.setSetpoint(lastXState.position);
        DriveConstants.AUTO_ALIGN_Y_CONTROLLER.setSetpoint(lastYState.position);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                DriveConstants.AUTO_ALIGN_X_CONTROLLER.calculate(pose.getX()),
                DriveConstants.AUTO_ALIGN_Y_CONTROLLER.calculate(pose.getY()),
                DriveConstants.AUTO_ALIGN_THETA_CONTROLLER.calculate(pose.getRotation().getRadians()),
                pose.getRotation());

        drive.driveRobotRelative(speeds);
    }

    @Override
    public boolean isFinished() {
        return DriveConstants.AUTO_ALIGN_X_CONTROLLER.atSetpoint()
                && DriveConstants.AUTO_ALIGN_Y_CONTROLLER.atSetpoint()
                && DriveConstants.AUTO_ALIGN_THETA_CONTROLLER.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.finished = true;

        this.drive.stop();
        VisionManager.defaultCameras();
    }
}