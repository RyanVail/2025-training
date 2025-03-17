package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoAlignConstants;
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

        SmartDashboard.putData("AlignXPID", AutoAlignConstants.X_CONTROLLER);
        SmartDashboard.putData("AlignYPID", AutoAlignConstants.Y_CONTROLLER);
        SmartDashboard.putData("AlignThetaPID", AutoAlignConstants.ANGLE_CONTROLLER);
    }

    public void setTarget(Pose2d target) {
        this.target = target;

        Pose2d start = drive.getPose();
        ChassisSpeeds speeds = drive.getRobotVelocity();

        if (!withinStartingDistance(start, target)) {
            Commands.print("Auto align dist too far. Start: " + start + " target: " + this.target).schedule();
            super.cancel();
            return;
        }

        Logger.recordOutput("AligningTo", target);

        XStateSetpoint = new State(target.getX(), 0);
        YStateSetpoint = new State(target.getY(), 0);

        lastXState = new State(start.getX(), speeds.vxMetersPerSecond);
        lastYState = new State(start.getY(), speeds.vyMetersPerSecond);

        AutoAlignConstants.X_CONTROLLER.reset();
        AutoAlignConstants.Y_CONTROLLER.reset();
        AutoAlignConstants.ANGLE_CONTROLLER.reset();

        AutoAlignConstants.ANGLE_CONTROLLER.setSetpoint(target.getRotation().getRadians());
    }

    /**
     * @param start The starting position.
     * @param target The position to align to.
     * @return True if the start and target poses are close enough to allow for aligning, false otherwise.
     */
    public boolean withinStartingDistance(Pose2d start, Pose2d target) {
        return start.getTranslation().getDistance(target.getTranslation()) <= AutoAlignConstants.MAX_DIST;
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

        lastXState = AutoAlignConstants.X_PROFILE.calculate(0.02, XStateSetpoint, lastXState);
        lastYState = AutoAlignConstants.Y_PROFILE.calculate(0.02, YStateSetpoint, lastYState);

        AutoAlignConstants.X_CONTROLLER.setSetpoint(lastXState.position);
        AutoAlignConstants.Y_CONTROLLER.setSetpoint(lastYState.position);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                AutoAlignConstants.X_CONTROLLER.calculate(pose.getX()),
                AutoAlignConstants.Y_CONTROLLER.calculate(pose.getY()),
                AutoAlignConstants.ANGLE_CONTROLLER.calculate(pose.getRotation().getRadians()),
                pose.getRotation());

        drive.driveRobotRelative(speeds);
    }

    @Override
    public boolean isFinished() {
        return drive.getPose().getTranslation().getDistance(target.getTranslation()) <= AutoAlignConstants.DIST_TOLERANCE
            && AutoAlignConstants.ANGLE_CONTROLLER.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.finished = true;

        this.drive.stop();
        VisionManager.defaultCameras();
    }
}