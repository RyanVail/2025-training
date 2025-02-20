package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

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
        this.setName("EndEffectorSEtAngle");
    }

    @Override
    public void initialize() {
        Commands.print("End effector set ange: " + angle).schedule();
        effector.setSetpoint(angle);
    }

    @Override
    public boolean isFinished() {
        Commands.print("EndEffectorSetAngle.isFinished(): " + (Math.abs(effector.getAngle() - angle) <= EndEffectorConstants.ALIGN_ANGLE)).schedule();
        return Math.abs(effector.getAngle() - angle) <= EndEffectorConstants.ALIGN_ANGLE;
    }
}
