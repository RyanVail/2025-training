package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorSetHeight extends Command {
    Elevator elevator;
    double height;

    public ElevatorSetHeight(Elevator elevator, double height) {
        super.addRequirements(elevator);
        this.elevator = elevator;
        this.height = height;
    }

    @Override
    public void initialize() {
        elevator.setSetpoint(height);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getHeight() - height) <= ElevatorConstants.ALIGN_DIST_METERS;
    }
}
