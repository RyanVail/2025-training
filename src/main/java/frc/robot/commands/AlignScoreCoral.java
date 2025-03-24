package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.LEDManager;
import frc.robot.VisionManager;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.LEDManager.Mode;
import frc.robot.subsystems.drive.Drive;

public class AlignScoreCoral extends AlignPose {
    Side side;
    CommandGenericHID controller;

    private Notifier rumble;

    public enum Side {
        LEFT,
        RIGHT,
    }

    public AlignScoreCoral(Drive drive, Side side, CommandGenericHID controller) {
        super(drive, null, AlignCamera.FRONT);
        this.controller = controller;
        this.side = side;
    }

    private void startRumble() {
        if (this.controller == null)
            return;

        if (this.rumble != null)
            this.rumble.close();

        controller.setRumble(RumbleType.kBothRumble, InputConstants.RUMBLE_VALUE);
        this.rumble = new Notifier(
                () -> {
                    controller.setRumble(RumbleType.kBothRumble, 0.0);
                });

        this.rumble.startSingle(InputConstants.RUMBLE_SECONDS);
    }

    @Override
    public void initialize() {
        LEDManager.setMode(Mode.AUTO_ALIGN_CORAL);

        super.setCameras();
        VisionManager.resetToCameraPose();
        Pose2d pose = drive.getPose();

        pose = (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
                ? FlippingUtil.flipFieldPose(pose)
                : pose;

        int index = getCoralScoreIndex(pose);

        Logger.recordOutput (
            "CoralAlignOffset",
            FieldConstants.REEF_TAG_POSITIONS[index / 2].minus(pose.getTranslation())
            .rotateBy(new Rotation2d(Units.degreesToRadians(60 * (index / 2))))
        );

        Pose2d align_pose = FieldConstants.CORAL_SCORE_POSES[index];
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            align_pose = FlippingUtil.flipFieldPose(align_pose);

        super.setTarget(new Target(align_pose));
        super.initialize();
    }

    /**
     * Gets the index of the reef coral segment to score on either the left or right
     * side of the robot at the suppplied pose.
     * 
     * @return The index of the coral reef segment to score on.
     */
    private int getCoralScoreIndex(Pose2d pose) {
        Translation2d robot_pos = pose.getTranslation();
        double closest_dist = Double.MAX_VALUE;
        int closest_pose = 0;

        // The closest algae position is found to determine which face of the reef's
        // hexagon the robot is aligned to so that a left and right button can be used
        // to align with the coral segment of the reef.
        for (int i = 0; i < FieldConstants.ALGAE_INTAKE_POSES.length; i++) {
            Pose2d p = FieldConstants.ALGAE_INTAKE_POSES[i];
            double dist = p.getTranslation().getDistance(robot_pos);
            if (closest_dist >= dist) {
                closest_dist = dist;
                closest_pose = i;
            }
        }

        return (closest_pose << 1) | (side == Side.LEFT ? 0 : 1);
    }

    @Override
    public void end(boolean interrupted) {
        LEDManager.stopMode(Mode.AUTO_ALIGN_CORAL);
        startRumble();

        super.end(interrupted);
    }
}
