package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LEDManager;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.LEDManager.Mode;
import frc.robot.subsystems.intake.Intake;

public class EjectCoral extends Command {
    Intake intake;
    double senseTime;

    public EjectCoral(Intake intake) {
        super.addRequirements(intake);
        this.intake = intake;
    }

    @Override
    public void initialize() {
        LEDManager.setMode(Mode.EJECT_CORAL);

        senseTime = 0;
        intake.setVoltage(IntakeConstants.CORAL_SCORE_VOLTAGE);
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
                : Units.millisecondsToSeconds(System.currentTimeMillis() - senseTime) >= IntakeConstants.CORAL_EJECT_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        LEDManager.stopMode(Mode.EJECT_CORAL);

        intake.setVoltage(0);
    }
}
