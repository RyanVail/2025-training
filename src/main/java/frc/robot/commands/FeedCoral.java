package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class FeedCoral extends Command {
    Intake intake;
    double sensePos;

    public FeedCoral(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;
        this.setName("FeedCoral");
    }

    @Override
    public void execute() {
        if (sensePos == Double.MAX_VALUE && intake.hasCoral())
            sensePos = intake.getPosition();
    }

    @Override
    public void initialize() {
        sensePos = Double.MAX_VALUE;
        intake.setVoltage(IntakeConstants.CORAL_FEED_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        if (sensePos == Double.MAX_VALUE)
            return false;

        return Robot.isSimulation()
                ? true
                : intake.getPosition() - sensePos >= IntakeConstants.CORAL_INTAKE_REV;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }
}
