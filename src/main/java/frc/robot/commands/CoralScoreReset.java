package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class CoralScoreReset extends Command {
    Elevator elevator;

    public CoralScoreReset(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
    }
}
