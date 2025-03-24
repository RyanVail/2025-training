package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.LEDManager;
import frc.robot.LEDManager.Mode;
import frc.robot.subsystems.intake.Intake;

public class StallAlgae extends Command {
    Intake intake;

    public StallAlgae(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;
    }

    @Override
    public void initialize() {
        LEDManager.setMode(Mode.STALL_ALGAE);

        intake.setVoltage(SmartDashboard.getNumber("AlgaeIntakeStallVolts", 0.0));
    }

    @Override
    public void end(boolean interrupted) {
        LEDManager.stopMode(Mode.STALL_ALGAE);

        intake.setVoltage(0);
    }
}
