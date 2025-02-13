package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.endeffector.EndEffector;

public class EndEffectorSetAngle extends Command {
    EndEffector effector;
    double angle;

    public EndEffectorSetAngle(EndEffector effector, double angle) {
        super.addRequirements(effector);
        this.effector = effector;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        effector.setSetpoint(angle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(effector.getAngle() - angle) <= EndEffectorConstants.ALIGN_ANGLE;
    }
}
