package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class FeedCoral extends Command {
    Intake intake;
    double senseTime;

    public FeedCoral(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;
        this.setName("FeedCoral");
    }

    @Override
    public void execute() {
        if (senseTime == 0 && intake.isCoralLoaded()) {
            senseTime = System.currentTimeMillis();
        }
    }

    @Override
    public void initialize() {
        senseTime = 0;
        intake.setVoltage(IntakeConstants.CORAL_FEED_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return Robot.isSimulation()
                ? true
                : ((System.currentTimeMillis() - senseTime) * 0.001) >= IntakeConstants.SENSE_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }
}
