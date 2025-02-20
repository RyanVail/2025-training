package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class FeedCoral extends Command {
    Intake intake;
    double timeSinceSense;

    public FeedCoral(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;
        this.setName("FeedCoral");
    }

    @Override
    public void execute() {
        // TODO: This is terrible
        if (intake.isCoralLoaded())
            timeSinceSense += 0.02;
    }

    @Override
    public void initialize() {
        timeSinceSense = 0;
        intake.setVoltage(IntakeConstants.CORAL_FEED_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return Robot.isSimulation() ? true : timeSinceSense >= IntakeConstants.SENSE_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        Commands.print("thehwoierwjweikjerwo").schedule();
        intake.setVoltage(0);
    }
}
