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
        io.addVisionEstimations(poses);
    }

    /**
     * @return Instance of TeleopCommand
     */
    public Command getTeleopCommand(Elevator elevator, CommandGenericHID controller) {
        return new TeleopCommand(this, elevator, controller);
    }

    /**
     * Finds a path to a pose and creates a command to follow that path.
     * 
     * @param pose The pose to align to.
     * @return The command to follow the found path.
     */
    public Command getPathCommand(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, DriveConstants.pathConstraints, 0.0);
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

    public ChassisSpeeds getCurrentSpeeds() {
        return io.getCurrentSpeeds();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return io.getRobotRelativeSpeeds();
    }

    public void drive(double x, double y, double omega) {
        this.io.drive(
                new ChassisSpeeds(
                        x * DriveConstants.MAX_SPEED,
                        y * DriveConstants.MAX_SPEED,
                        omega * DriveConstants.MAX_SPEED));
    }

    public void drive(ChassisSpeeds speeds) {
        this.io.drive(speeds);
    }

    public void driveRobotRelative(double x, double y, double omega) {
        this.io.drive(new ChassisSpeeds(x * DriveConstants.MAX_SPEED, y * DriveConstants.MAX_SPEED, omega));
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.io.drive(speeds);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        
    }
}
