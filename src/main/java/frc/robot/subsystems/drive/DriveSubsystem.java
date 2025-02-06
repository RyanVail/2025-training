package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopCommand;

public class DriveSubsystem extends SubsystemBase {
    public class PoseSupplier implements Supplier<Pose2d> {
        public DriveSubsystem drive;

        public PoseSupplier(DriveSubsystem drive) {
            this.drive = drive;
        }

        public Pose2d get() {
            return drive.getPose();
        }
    }

    private DriveSubsystemIO io;

    public DriveSubsystem(DriveSubsystemIO io) {
        this.io = io;

        Commands.print("Drive subsystem is init");
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
                DriveConstants.driveController,
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
        Logger.recordOutput("SwerveStates", io.getSwerveStates());
    }

    /**
     * @return Instance of TeleopCommand
     */
    public Command getTeleopCommand(CommandGenericHID controller) {
        return new TeleopCommand(this, controller);
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
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        x * DriveConstants.MAX_SPEED,
                        y * DriveConstants.MAX_SPEED,
                        omega,
                        io.getRotation()));
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
}
