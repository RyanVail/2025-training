package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.flywheel.Flywheel;

public class EjectCoral extends Command {
    Flywheel flywheel;

    public EjectCoral(Flywheel flywheel) {
        super.addRequirements(flywheel);
        this.flywheel = flywheel;

        flywheel.setVelocitySetpoint(FlywheelConstants.CORAL_SCORE_VEL);
    }

    @Override
    public boolean isFinished() {
        return Robot.isSimulation() ? true : !flywheel.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocitySetpoint(0);
    }
}
