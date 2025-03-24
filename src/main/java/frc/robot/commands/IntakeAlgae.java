package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LEDManager;
import frc.robot.Constants.IntakeConstants;
import frc.robot.LEDManager.Mode;
import frc.robot.subsystems.intake.Intake;

public class IntakeAlgae extends Command {
    Intake intake;
    boolean started;

    public IntakeAlgae(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;

        SmartDashboard.putNumber("AlgaeIntakeStallVolts", IntakeConstants.ALGAE_STALL_VOLTAGE);
        SmartDashboard.putNumber("AlgaeIntakeStallVel", IntakeConstants.ALGAE_INTAKE_STALL_VEL);
        SmartDashboard.putNumber("AlgaeIntakeVolts", IntakeConstants.ALGAE_INTAKE_VOLTAGE);
        SmartDashboard.putNumber("AlgaeIntakeStartVel", IntakeConstants.ALGAE_INTAKE_START_VEL);
    }

    @Override
    public void initialize() {
        LEDManager.setMode(Mode.INTAKE_ALGAE);

        intake.setVoltage(SmartDashboard.getNumber("AlgaeIntakeVolts", 0.0));
        started = false;
    }

    @Override
    public void execute() {
        if (!started) {
            started = intake.getVelocity() >= SmartDashboard.getNumber("AlgaeIntakeStartVel", 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return started && intake.getVelocity() <= SmartDashboard.getNumber("AlgaeIntakeStallVel", 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        LEDManager.stopMode(Mode.INTAKE_ALGAE);

        // TODO: This is terribe but I can't find a better way to do this.
        new StallAlgae(intake).schedule();
        intake.setVoltage(0);
    }
}
