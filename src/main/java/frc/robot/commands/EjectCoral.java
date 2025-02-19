package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class EjectCoral extends Command {
    Intake intake;

    public EjectCoral(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;

        intake.setVoltage(IntakeConstants.CORAL_SCORE_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return Robot.isSimulation() ? true : !intake.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }
}
