package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Robot;
import frc.robot.subsystems.flywheel.Flywheel;

public class FeedCoral extends Command {
    Flywheel flywheel;

    public FeedCoral(Flywheel flywheel) {
        super.addRequirements(flywheel);
        this.flywheel = flywheel;

        flywheel.setVelocitySetpoint(FlywheelConstants.CORAL_FEED_VEL);
    }

    @Override
    public boolean isFinished() {
        return Robot.isSimulation() ? true : flywheel.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocitySetpoint(0);
    }
}
