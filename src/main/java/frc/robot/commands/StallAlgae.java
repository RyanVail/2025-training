package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class StallAlgae extends Command {
    Intake intake;

    public StallAlgae(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setVoltage(IntakeConstants.ALGAE_STALL_VOLTAGE);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }
}
