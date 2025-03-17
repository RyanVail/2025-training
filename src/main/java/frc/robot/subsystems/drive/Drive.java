package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.VisionManager;

public class Drive extends SubsystemBase {
    public class PoseSupplier implements Supplier<Pose2d> {
        public Drive drive;

        public PoseSupplier(Drive drive) {
            this.drive = drive;
        }

        public Pose2d get() {
            return drive.getPose();
        }
    }

    private DriveIO io;
    private Rotation2d gyroOffset;

    public static final String LPREFIX = "/Subsystems/Drive/";

    public Drive(DriveIO io) {
        this.io = io;

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                DriveConstants.PPDriveController,
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return (alliance.isPresent())
                            ? alliance.get() == DriverStation.Alliance.Red
                            : false;
                },
                this);

        this.io.resetPose(new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d()));
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LPREFIX + "Pose", io.getPose());
        Logger.recordOutput(LPREFIX + "FlippedPose", FlippingUtil.flipFieldPose(io.getPose()));
        Logger.recordOutput(LPREFIX + "SwerveStates", io.getSwerveStates());

        io.periodic();

        EstimatedRobotPose[] poses = VisionManager.getEstimatedPoses();
        for (int i = 0; i < poses.length; i++) {
            if (poses[i] == null)
                continue;

            Logger.recordOutput("_EstimatedPose" + i, poses[i].estimatedPose);
        }

        io.addVisionEstimations(poses);
    }

    /**
     * @return Instance of TeleopCommand
     */
    public Command getTeleopCommand(Elevator elevator, CommandGenericHID controller) {
        return new TeleopCommand(this, elevator, controller);
    }

    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }   

    public Supplier<Pose2d> getPoseSupplier() {
        return new PoseSupplier(this);
    }

    public Pose2d getPose() {
        return io.getPose();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return io.getRobotRelativeSpeeds();
    }

    public void driveRobotRelative(double x, double y, double omega) {
        this.driveRobotRelative(new ChassisSpeeds(x, y, omega));
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        // TODO: Remove.
        // String s = "-----\nspeed: " + speeds;
        // var trace = Thread.currentThread().getStackTrace();
        // for (var t : trace) { s = s + t + "\n"; }

        // Commands.print(s).schedule();

        this.io.drive(speeds);
    }

    public void setGryoOffset(Rotation2d rotation) {
        this.gyroOffset = rotation;
    }

    public void resetGyroOffset() {
        this.gyroOffset = this.io.getGyroRotation();
    }

    public Rotation2d getGyroRotation() {
        return this.io.getGyroRotation().minus(gyroOffset);
    }

    public ChassisSpeeds getRobotVelocity() {
        return io.getRobotVelocity();
    }

    public void stop() {
        this.driveRobotRelative(new ChassisSpeeds());
    }
}
