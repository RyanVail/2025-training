package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.LEDManager.Mode;
import frc.robot.LEDManager;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class IntakeCoral extends Command {
    Intake intake;
    double sensePos;

    public IntakeCoral(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;
    }

    @Override
    public void execute() {
        if (sensePos == Double.MAX_VALUE && intake.hasCoral())
            sensePos = intake.getPosition();
    }

    @Override
    public void initialize() {
        LEDManager.setMode(Mode.INTAKE_CORAL);

        sensePos = Double.MAX_VALUE;
        intake.setVoltage(IntakeConstants.CORAL_INTAKE_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        if (sensePos == Double.MAX_VALUE)
            return false;

        return Robot.isSimulation()
                ? true
                : intake.getPosition() - sensePos >= IntakeConstants.CORAL_INTAKE_REV;
    }

    @Override
    public void end(boolean interrupted) {
        LEDManager.stopMode(Mode.INTAKE_CORAL);

        intake.setVoltage(0);
    }
}
