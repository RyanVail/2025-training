package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeAlgae extends Command {
    Intake intake;
    boolean stalled;

    public IntakeAlgae(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;
        this.setName("IntakeAlgae");
    }

    @Override
    public void execute() {
        // if (!stalled && intake.getVelocity() <= IntakeConstants.ALGAE_INTAKE_STALL_VEL)
        //     intake.setVoltage(IntakeConstants.ALGAE_STALL_VOLTAGE);

        Logger.recordOutput("Intake Velocity", intake.getVelocity());
    }

    @Override
    public void initialize() {
        intake.setVoltage(IntakeConstants.ALGAE_INTAKE_VOLTAGE);
        stalled = false;
    }

    @Override
    public boolean isFinished() {
        return stalled && intake.getVelocity() >= IntakeConstants.ALGAE_INTAKE_UNSTALL_VEL;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }
}
