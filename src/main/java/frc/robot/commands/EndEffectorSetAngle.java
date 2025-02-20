package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;

public class EndEffectorSetAngle extends Command {
    EndEffector effector;
    Elevator elevator;
    double angle;

    public EndEffectorSetAngle(EndEffector effector, Elevator elevator, double angle) {
        super.addRequirements(effector);
        this.effector = effector;
        this.elevator = elevator;
        this.angle = angle;
    }

    @Override
    public void execute() {
        if (elevator.getHeight() >= EndEffectorConstants.REQUIRED_ELEVATOR_HEIGHT
        || angle <= EndEffectorConstants.MIN_ELEVATOR_REQUIRED_ANGLE)
            effector.setSetpoint(angle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(effector.getAngle() - angle) <= EndEffectorConstants.ALIGN_ANGLE;
    }
}
