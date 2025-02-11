package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.flywheel.Flywheel;

public class FeedCoral extends Command {
    Flywheel flywheel;

    public FeedCoral(Flywheel flywheel) {
        this.flywheel = flywheel;

        this.flywheel.setVelocity(FlywheelConstants.CORAL_FEED_VEL);
    }

    @Override
    public boolean isFinished() {
        return flywheel.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocity(0);
    }
}
