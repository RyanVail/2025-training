package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    @Override
    public void execute()
    {
        Logger.recordOutput("AligningTo", pose);
        ChassisSpeeds speeds = (driveController.calculate(
            drive.getPose(),
            pose,
            0,
            pose.getRotation()
        ));
        // if(Math.abs(speeds.vxMetersPerSecond) <=


        speeds.vxMetersPerSecond = x.calculate(speeds.vxMetersPerSecond);
        speeds.vyMetersPerSecond = y.calculate(speeds.vyMetersPerSecond);
        drive.driveRobotRelative(speeds);
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