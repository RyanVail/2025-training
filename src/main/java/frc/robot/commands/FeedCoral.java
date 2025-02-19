package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class FeedCoral extends Command {
    Intake intake;

    public FeedCoral(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;

        intake.setVoltage(IntakeConstants.CORAL_FEED_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return Robot.isSimulation() ? true : intake.isCoralLoaded();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }
}
