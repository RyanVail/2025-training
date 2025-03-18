package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class SlowStop extends Command {
    Drive drive;

    State lastXState;
    State lastYState;

    State XStateSetpoint;
    State YStateSetpoint;

    public SlowStop(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        Pose2d pose = drive.getPose();
        ChassisSpeeds speeds = drive.getRobotVelocity();

        XStateSetpoint = new State(pose.getX(), 0.0);
        YStateSetpoint = new State(pose.getY(), 0.0);

        lastXState = new State(pose.getX(), speeds.vxMetersPerSecond);
        lastYState = new State(pose.getY(), speeds.vyMetersPerSecond);
    }

    @Override
    public void execute() {
        lastXState = DriveConstants.SLOW_STOP_X_PROFILE.calculate(0.02, XStateSetpoint, lastXState);
        lastYState = DriveConstants.SLOW_STOP_Y_PROFILE.calculate(0.02, YStateSetpoint, lastYState);

        drive.driveRobotRelative(new ChassisSpeeds(lastXState.velocity, lastYState.velocity, 0.0));
    }
}
