package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class SlowStop extends Command {
    Drive drive;

    public SlowStop(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        ChassisSpeeds speeds = drive.getRobotVelocity();

        DriveConstants.X_SLEW_RATE.reset(speeds.vxMetersPerSecond);
        DriveConstants.Y_SLEW_RATE.reset(speeds.vyMetersPerSecond);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = drive.getRobotVelocity();
        drive.driveRobotRelative (
            new ChassisSpeeds (
                DriveConstants.X_SLEW_RATE.calculate(0.0),
                DriveConstants.Y_SLEW_RATE.calculate(0.0),
                0.0
            )
        );
    }
}
