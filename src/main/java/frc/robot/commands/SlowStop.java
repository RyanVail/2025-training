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

        DriveConstants.SLOW_STOP_X_SLEW.reset(speeds.vxMetersPerSecond);
        DriveConstants.SLOW_STOP_Y_SLEW.reset(speeds.vyMetersPerSecond);
    }

    @Override
    public void execute() {
        drive.driveRobotRelative (
            new ChassisSpeeds (
                DriveConstants.SLOW_STOP_X_SLEW.calculate(0.0),
                DriveConstants.SLOW_STOP_Y_SLEW.calculate(0.0),
                0.0
            )
        );
    }
}
