package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.flywheel.Flywheel;

public class ScoreCoral extends SequentialCommandGroup {
    EndEffector effector;
    ElevatorSubsystem elevator;
    Flywheel flywheel;

    private int findClosestLevel() {
        double height = elevator.getHeight() - ElevatorConstants.CORAL_SCORE_OFFSET;
        int closest_level = 0;
        double closest_dist = Double.MAX_VALUE;
        for (int l = 0; l < FieldConstants.CORAL_LEVEL_HEIGHTS.length; l++) {
            double dist = Math.abs(height - FieldConstants.CORAL_LEVEL_HEIGHTS[l]);
            if (dist <= closest_dist) {
                closest_dist = dist;
                closest_level = l;
            }
        }

        return closest_level;
    }

    public ScoreCoral(EndEffector effector, ElevatorSubsystem elevator, Flywheel flywheel) {
        super.addRequirements(effector, elevator, flywheel);
        this.effector = effector;
        this.elevator = elevator;
        this.flywheel = flywheel;

        int level = findClosestLevel();
        super.addCommands(
                new EndEffectorSetAngle(effector, EndEffectorConstants.SCORING_ANGLES[level]),
                new EjectCoral(flywheel),
                new EndEffectorSetAngle(effector, EndEffectorConstants.IDLE_ANGLE));
    }
}
