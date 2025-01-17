package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TeleopCommand extends Command {
    private DriveSubsystem drive;
    private CommandGenericHID controller;

    public TeleopCommand(DriveSubsystem drive, CommandGenericHID controller) {
        super.addRequirements(drive);
        this.drive = drive;
        this.controller = controller;
    }

    private double processAxis(double axis)
    {
        boolean neg = axis <= 0;
        axis = Math.abs(axis);

        // Apply deadzone.
        axis = (axis > DriveConstants.DEADZONE) ? axis : 0;

        // Make the drive speed exponential.
        return (neg ? -1 : 1) * Math.pow(axis, 1.4);
    }

    @Override
    public void execute() {
        drive.drive(
                processAxis(controller.getRawAxis(1)),
                processAxis(controller.getRawAxis(0)),
                processAxis(controller.getRawAxis(4)));
    }
}
