package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorSetHeight extends Command {
    ElevatorSubsystem elevator;
    double height;

    public ElevatorSetHeight(ElevatorSubsystem elevator, double height) {
        super.addRequirements(elevator);
        this.elevator = elevator;
        this.height = height;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getHeight()) <= ElevatorConstants.ELEVATOR_ALIGN_DIST;
    }
}
