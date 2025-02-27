package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class AlignPose extends Command {
    Drive drive;
    Pose2d pose;
    HolonomicDriveController driveController;

    public AlignPose(Drive drive, Pose2d pose) {
        addRequirements(drive);

        this.drive = drive;
        this.pose = pose;
        this.driveController = DriveConstants.driveController;
    }

    SlewRateLimiter x = new SlewRateLimiter(10);
    SlewRateLimiter y = new SlewRateLimiter(10);
    
    PIDController xController = new PIDController(9.0, 0.0, 0.0);
    PIDController yController = new PIDController(9.0, 0.0, 0.0);
    PIDController angleController = new PIDController(4.0, 0, 0);

    @Override
    public void execute()
    {
        Logger.recordOutput("AligningTo", pose);

        xController.setSetpoint(pose.getX());
        yController.setSetpoint(pose.getY());
        angleController.setSetpoint(pose.getRotation().getRadians());
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        ChassisSpeeds speeds2 = new ChassisSpeeds();
        speeds2.vxMetersPerSecond = xController.calculate(drive.getPose().getX());
        speeds2.vyMetersPerSecond = yController.calculate(drive.getPose().getY());
        speeds2.omegaRadiansPerSecond = angleController.calculate(drive.getPose().getRotation().getRadians());

        // speeds.vxMetersPerSecond = x.calculate(speeds.vxMetersPerSecond);
        // speeds.vyMetersPerSecond = y.calculate(speeds.vyMetersPerSecond);
        drive.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds2, drive.getPose().getRotation()));
    }

    @Override
    public boolean isFinished()
    {
        return driveController.atReference();
    }

    @Override
    public void end(boolean interrupted)
    {
        drive.driveRobotRelative(new ChassisSpeeds());
    }
}