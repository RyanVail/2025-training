package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class EjectCoral extends Command {
    Intake intake;
    double timeSinceSense;

    public EjectCoral(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;
    }

    @Override
    public void initialize() {
        Commands.print("Initing eject coral").schedule();
        timeSinceSense = 0;
        intake.setVoltage(IntakeConstants.CORAL_SCORE_VOLTAGE);
    }

    @Override
    public void execute() {
        if (!intake.hasCoral())
            timeSinceSense += 0.02;
    }

    @Override
    public boolean isFinished() {
        return Robot.isSimulation() ? true : timeSinceSense >= IntakeConstants.SENSE_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }
}
