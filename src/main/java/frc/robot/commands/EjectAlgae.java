package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class EjectAlgae extends Command {
    double senseTime;
    Intake intake;

    public EjectAlgae(Intake intake) {
        super.addRequirements(intake);

        this.intake = intake;
    }

    @Override
    public void initialize() {
        senseTime = 0;

        if (intake.hasCoral() || intake.isCoralLoaded())
            super.cancel();

        intake.setVoltage(IntakeConstants.ALGAE_SCORE_VOLTAGE);
        senseTime = System.currentTimeMillis();
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

        return Robot.isSimulation()
                ? true
                : ((System.currentTimeMillis() - senseTime) * 0.001) >= IntakeConstants.ALGAE_EJECT_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }
}
