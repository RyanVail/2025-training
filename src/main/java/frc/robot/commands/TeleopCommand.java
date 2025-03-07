package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class TeleopCommand extends Command {
    Drive drive;
    Elevator elevator;
    CommandGenericHID controller;

    public TeleopCommand(Drive drive, Elevator elevator, CommandGenericHID controller) {
        super.addRequirements(drive);
        this.drive = drive;
        this.elevator = elevator;
        this.controller = controller;

        SmartDashboard.putNumber("ControlPow", 3);
    }

    private double processAxis(double axis) {
        boolean neg = axis <= 0;
        axis = Math.abs(axis);

        // Apply deadzone.
        if (axis <= DriveConstants.DEADZONE)
            return 0;

        // Make the drive speed exponential.
        axis = (neg ? -1 : 1) * Math.pow(axis, SmartDashboard.getNumber("ControlPow", 3));

        // Slow the speed based on the elevator height.
        return axis;
    }

    @Override
    public void execute() {
        double x = processAxis(-controller.getRawAxis(1));
        double y = processAxis(-controller.getRawAxis(0));
        double yaw = processAxis(-controller.getRawAxis(4));

        for (int i = 0; i < DriveConstants.X_SLEW_LIMITERS.length; i++) {
            DriveConstants.X_SLEW_LIMITERS[i].calculate(x);
        }

        for (int i = 0; i < DriveConstants.Y_SLEW_LIMITERS.length; i++) {
            DriveConstants.Y_SLEW_LIMITERS[i].calculate(y);
        }

        for (int i = 0; i < DriveConstants.ROT_SLEW_LIMITERS.length; i++) {
            DriveConstants.ROT_SLEW_LIMITERS[i].calculate(yaw);
        }

        for (int i = DriveConstants.HEIGHT_LEVELS.length - 1; i >= 0; i--) {
            if (DriveConstants.HEIGHT_LEVELS[i] <= elevator.getHeight()) {
                x = DriveConstants.X_SLEW_LIMITERS[i].lastValue();
                y = DriveConstants.Y_SLEW_LIMITERS[i].lastValue();
                yaw = DriveConstants.ROT_SLEW_LIMITERS[i].lastValue();

                x = MathUtil.clamp(x, -DriveConstants.MAX_SPEEDS[i], DriveConstants.MAX_SPEEDS[i]);
                y = MathUtil.clamp(y, -DriveConstants.MAX_SPEEDS[i], DriveConstants.MAX_SPEEDS[i]);
                yaw = MathUtil.clamp(yaw, -DriveConstants.MAX_SPEEDS[i], DriveConstants.MAX_SPEEDS[i]);

                break;
            }
        }

        drive.drive(x, y, yaw);
    }
}
