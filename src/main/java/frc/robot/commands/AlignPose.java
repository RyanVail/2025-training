package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class AlignPose extends Command {
    Drive drive;
    Pose2d pose;

    public AlignPose(Drive drive, Pose2d pose) {
        addRequirements(drive);

        this.drive = drive;
        this.pose = pose;

        xController.setTolerance(DriveConstants.MIN_ALIGN_DIST);
        yController.setTolerance(DriveConstants.MIN_ALIGN_DIST);
        angleController.setTolerance(DriveConstants.MIN_ALIGN_ANGLE);
    }

    SlewRateLimiter x = new SlewRateLimiter(10);
    SlewRateLimiter y = new SlewRateLimiter(10);

    PIDController xController = new PIDController(9.0, 0.0, 0.0);
    PIDController yController = new PIDController(9.0, 0.0, 0.0);
    PIDController angleController = new PIDController(4.0, 0, 0);

    @Override
    public void execute() {
        if (pose.getTranslation().getDistance(drive.getPose().getTranslation()) >= DriveConstants.AUTO_ALIGN_MAX_DIST)
            super.cancel();

        xController.setSetpoint(pose.getX());
        yController.setSetpoint(pose.getY());
        angleController.setSetpoint(pose.getRotation().getRadians());
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        ChassisSpeeds speeds = new ChassisSpeeds();
        speeds.vxMetersPerSecond = xController.calculate(drive.getPose().getX());
        speeds.vyMetersPerSecond = yController.calculate(drive.getPose().getY());
        speeds.omegaRadiansPerSecond = angleController.calculate(drive.getPose().getRotation().getRadians());
        drive.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getPose().getRotation()));
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
    }
}