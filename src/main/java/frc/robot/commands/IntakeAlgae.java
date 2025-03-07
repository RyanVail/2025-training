package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeAlgae extends Command {
    Intake intake;
    boolean started;

    public IntakeAlgae(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setVoltage(IntakeConstants.ALGAE_INTAKE_VOLTAGE);
        started = false;
    }

    @Override
    public void execute() {
        if (!started) {
            started = intake.getVelocity() >= IntakeConstants.ALGAE_INTAKE_START_VEL;
        }
    }

    @Override
    public boolean isFinished() {
        return started && intake.getVelocity() <= IntakeConstants.ALGAE_INTAKE_STALL_VEL;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: This is terribe but I can't find a better way to do this.
        new StallAlgae(intake).schedule();
        intake.setVoltage(0);
    }
}
