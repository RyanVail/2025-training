package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorSetHeight extends Command {
    Elevator elevator;
    double height;

    public ElevatorSetHeight(Elevator elevator, double height) {
        super.addRequirements(elevator);
        this.elevator = elevator;
        this.height = height;
        this.setName("Elevator Set Height");
    }

    @Override
    public void initialize() {
        elevator.setSetpoint(height);
    }

    @Override
    public void execute() {
        Commands.print("ElevatorSetHeight.execute()").schedule();
    }

    @Override
    public void end(boolean interrupted) {
        Commands.print("ElevatorSetHeight ended, " + (interrupted ? "interrupted" : "not interrupted")).schedule();
    }

    @Override
    public boolean isFinished() {
        Commands.print("Dif" + String.valueOf(elevator.getHeight() - height)).schedule();
        Commands.print("ElevatorSetHeigt.isFinished(): " + String.valueOf(Math.abs(elevator.getHeight() - height) <= ElevatorConstants.ALIGN_DIST_METERS)).schedule();
        return Math.abs(elevator.getHeight() - height) <= ElevatorConstants.ALIGN_DIST_METERS;
    }
}
