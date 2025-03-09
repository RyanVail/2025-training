package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionManager;

public class AlignPose extends Command {
    protected Pose2d target_pose;
    Drive drive;
    AlignCamera camera;

    PIDController xController = new PIDController(4.0, 0.0, 0.0);
    PIDController yController = new PIDController(4.0, 0.0, 0.0);
    PIDController angleController = new PIDController(2.0, 0, 0);

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
        this.camera = camera;

        SmartDashboard.putData("AutoAlignX", xController);
        SmartDashboard.putData("AutoAlignY", yController);
        SmartDashboard.putData("AutoAlignAngle", angleController);

        // TODO: Make a constant.
        xController.setTolerance(Units.inchesToMeters(0.15));
        yController.setTolerance(Units.inchesToMeters(0.15));
        angleController.setTolerance(Units.inchesToMeters(0.15));
    }

    public void initialize() {
        Pose2d robot_pose = drive.getPose();

        // Ensuring the target position is within an acceptable distance.
        double dist = robot_pose.getTranslation().getDistance(target_pose.getTranslation());
        if (dist >= DriveConstants.AUTO_ALIGN_MAX_DIST) {
            super.cancel();
            return;
        }
    }

    @Override
    public void execute() {
        Pose2d drive_pose = drive.getPose();
        if (target_pose.getTranslation()
                .getDistance(drive_pose.getTranslation()) >= DriveConstants.AUTO_ALIGN_MAX_DIST) {
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

        Logger.recordOutput("AligningTo", target_pose);

        xController.setSetpoint(target_pose.getX());
        yController.setSetpoint(target_pose.getY());
        angleController.setSetpoint(target_pose.getRotation().getRadians());
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        ChassisSpeeds speed = new ChassisSpeeds();
        speed.vxMetersPerSecond = xController.calculate(drive.getPose().getX());
        speed.vyMetersPerSecond = yController.calculate(drive.getPose().getY());
        speed.omegaRadiansPerSecond = angleController.calculate(drive.getPose().getRotation().getRadians());

        drive.driveFieldRelative(speed);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint()
                && yController.atSetpoint()
                && angleController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        VisionManager.onlyFront(); // TODO: There should be a normal one.
    }
}