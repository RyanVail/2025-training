// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemIOSwerve;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystemIOSim;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;

public class RobotContainer {
    CommandGenericHID commandGenericHID;

    DriveSubsystem drive;
    Flywheel flywheel;
    ElevatorSubsystem elevator;

    public RobotContainer() {
        commandGenericHID = new CommandGenericHID(Constants.CONTROLLER_PORT);

        if (Robot.isSimulation()) {
            drive = new DriveSubsystem(new DriveSubsystemIOSwerve());
            flywheel = new Flywheel(new FlywheelIOSim());
            elevator = new ElevatorSubsystem(new ElevatorSubsystemIOSim());
        } else {
            drive = new DriveSubsystem(new DriveSubsystemIOSwerve());
        }

        configureBindings();
        configureAuto();
    }

    private void configureAuto() {
        PathManager.configurePaths();
    }

    /**
     * This method will be used to configure controls
     */
    private void configureBindings() {
        // commandGenericHID.button(1).onTrue(flywheel.setVelocity(1000));
        // commandGenericHID.button(1).onFalse(flywheel.setVelocity(0));

        // commandGenericHID.button(2).onTrue(elevator.setHeight(0.25));
        // commandGenericHID.button(2).onFalse(elevator.setHeight(0.75));

        /*
         * Set the drive subsystem to use the command returned by getTeleopCommand
         * if no other command is scheduled for the subsystem
         */
        drive.setDefaultCommand(drive.getTeleopCommand(commandGenericHID));
    }
}
