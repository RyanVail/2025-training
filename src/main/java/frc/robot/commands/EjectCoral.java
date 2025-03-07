package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class EjectCoral extends Command {
    Intake intake;
    double senseTime;
    boolean algae; // TODO: Rename command to just eject.

    public EjectCoral(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;
    }

    @Override
    public void initialize() {
        senseTime = 0;
        algae = !intake.hasCoral();

        if (algae) {
            intake.setVoltage(IntakeConstants.ALGAE_SCORE_VOLTAGE);
            senseTime = System.currentTimeMillis();
        } else {
            intake.setVoltage(IntakeConstants.CORAL_SCORE_VOLTAGE);
        }
    }

    @Override
    public void execute() {
        if (senseTime == 0 && !intake.hasCoral()) {
            senseTime = System.currentTimeMillis();
        }
    }

    @Override
    public boolean isFinished() {
        if (senseTime == 0)
            return false;

        double time = (algae) ? IntakeConstants.ALGAE_EJECT_TIME : IntakeConstants.SENSE_TIME;
        return Robot.isSimulation()
                ? true
                : ((System.currentTimeMillis() - senseTime) * 0.001) >= time;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }
}
