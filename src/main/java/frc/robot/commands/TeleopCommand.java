package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class TeleopCommand extends Command {
    private Drive drive;
    private Elevator elevator;
    private CommandGenericHID controller;

    public TeleopCommand(Drive drive, Elevator elevator, CommandGenericHID controller) {
        super.addRequirements(drive, elevator);
        this.drive = drive;
        this.elevator = elevator;
        this.controller = controller;

        SmartDashboard.putNumber("ControlPow", 1.8);
    }

    private double processAxis(double axis) {
        boolean neg = axis <= 0;
        axis = Math.abs(axis);

        // Apply deadzone.
        axis = (axis > DriveConstants.DEADZONE) ? axis : 0;

        // Make the drive speed exponential.
        axis = (neg ? -1 : 1) * Math.pow(axis, SmartDashboard.getNumber("ControlPow", 1.8));

        // Slow the speed based on the elevator height.
        return axis * (1 - (this.elevator.getSetpoint() / ElevatorConstants.MAX_HEIGHT)
                * DriveConstants.ELEVATOR_HEIGHT_DIMMER);
    }

    @Override
    public void execute() {
        drive.drive(
                processAxis(controller.getRawAxis(1)),
                processAxis(controller.getRawAxis(0)),
                processAxis(controller.getRawAxis(4) * 5));
    }
}
