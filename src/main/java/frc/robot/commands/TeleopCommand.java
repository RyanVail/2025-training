package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class TeleopCommand extends Command {
    Drive drive;
    Elevator elevator;
    CommandGenericHID controller;
    double last_x;
    double last_y;
    double last_yaw;
    int last_slew;

    private Notifier rumble;

    public TeleopCommand(Drive drive, Elevator elevator, CommandGenericHID controller) {
        super.addRequirements(drive);
        this.drive = drive;
        this.elevator = elevator;
        this.controller = controller;

        SmartDashboard.putNumber("ControlPow", 3);
    }

    private void startRumble() {
        if (this.rumble != null)
            this.rumble.close();

        controller.setRumble(RumbleType.kBothRumble, InputConstants.RUMBLE_VALUE);
        this.rumble = new Notifier(
                () -> {
                    controller.setRumble(RumbleType.kBothRumble, 0.0);
                });

        this.rumble.startSingle(InputConstants.RUMBLE_SECONDS);
    }

    public void initialize() {
        last_x = 0.0;
        last_y = 0.0;
        last_yaw = 0.0;
        last_slew = -1;

        startRumble();
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
        if (DriverStation.isAutonomous()) {
            this.drive.stop();
            return;
        }

        double x = -processAxis(-controller.getRawAxis(1)); // 5
        double y = -processAxis(-controller.getRawAxis(0)); // 4
        double yaw = processAxis(-controller.getRawAxis(4)); // 0

        for (int i = DriveConstants.HEIGHT_LEVELS.length - 1; i >= 0; i--) {
            if (DriveConstants.HEIGHT_LEVELS[i] <= elevator.getHeight()) {
                if (last_slew != i) {
                    DriveConstants.X_SLEW_LIMITERS[i].reset(last_x);
                    DriveConstants.Y_SLEW_LIMITERS[i].reset(last_y);
                    DriveConstants.ROT_SLEW_LIMITERS[i].reset(last_yaw);
                }

                x = MathUtil.clamp(x, -DriveConstants.MAX_SPEEDS[i], DriveConstants.MAX_SPEEDS[i]);
                y = MathUtil.clamp(y, -DriveConstants.MAX_SPEEDS[i], DriveConstants.MAX_SPEEDS[i]);
                yaw = MathUtil.clamp(yaw, -DriveConstants.MAX_SPEEDS[i], DriveConstants.MAX_SPEEDS[i]);

                x = DriveConstants.X_SLEW_LIMITERS[i].calculate(x);
                y = DriveConstants.Y_SLEW_LIMITERS[i].calculate(y);
                yaw = DriveConstants.ROT_SLEW_LIMITERS[i].calculate(yaw);

                last_slew = i;
                break;
            }
        }

        drive.driveGyroRelative(
                -x * DriveConstants.MAX_SPEED,
                -y * DriveConstants.MAX_SPEED,
                yaw * DriveConstants.MAX_SPEED);

        last_x = x;
        last_y = y;
        last_yaw = yaw;
    }

    @Override
    public void end(boolean interrupted) {
        startRumble();
    }
}