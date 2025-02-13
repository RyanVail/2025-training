package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AutoFeedCoral;
import frc.robot.commands.AutoScoreCoral;
import frc.robot.commands.ElevatorSetHeight;
import frc.robot.commands.EndEffectorSetAngle;
import frc.robot.commands.FeedCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemIOSwerve;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystemIOSim;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOSpark;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemIOSim;

public class RobotContainer {
    CommandGenericHID commandGenericHID;

    DriveSubsystem drive;
    Flywheel flywheel;
    ElevatorSubsystem elevator;
    VisionSubsystem vision;
    EndEffector endEffector;

    public RobotContainer() {
        commandGenericHID = new CommandGenericHID(Constants.CONTROLLER_PORT);

        if (Robot.isSimulation()) {
            drive = new DriveSubsystem(new DriveSubsystemIOSwerve());
            flywheel = new Flywheel(new FlywheelIOSim());
            elevator = new ElevatorSubsystem(new ElevatorSubsystemIOSim());
            vision = new VisionSubsystem(new VisionSubsystemIOSim(), drive.getPoseSupplier());
            endEffector = new EndEffector(new EndEffectorIOSim(), elevator.getRealMech(), elevator.getSetpointMech());
        } else {
            drive = new DriveSubsystem(new DriveSubsystemIOSwerve());
            // endEffector = new EndEffector(new EndEffectorIOSpark(), elevator.getRealMech(), elevator.getSetpoint());
        }

        configureBindings();
        configureAuto();
    }

    private void configureAuto() {
        NamedCommands.registerCommand("ElevatorL1", new ElevatorSetHeight(elevator,
                ElevatorConstants.CORAL_SCORE_OFFSET + FieldConstants.CORAL_LEVEL_HEIGHTS[0]));
        NamedCommands.registerCommand("ElevatorL2", new ElevatorSetHeight(elevator,
                ElevatorConstants.CORAL_SCORE_OFFSET + FieldConstants.CORAL_LEVEL_HEIGHTS[1]));
        NamedCommands.registerCommand("ElevatorL3", new ElevatorSetHeight(elevator,
                ElevatorConstants.CORAL_SCORE_OFFSET + FieldConstants.CORAL_LEVEL_HEIGHTS[2]));
        NamedCommands.registerCommand("ElevatorL4", new ElevatorSetHeight(elevator,
                ElevatorConstants.CORAL_SCORE_OFFSET + FieldConstants.CORAL_LEVEL_HEIGHTS[3]));
        NamedCommands.registerCommand("ElevatorIntake",
                new ElevatorSetHeight(elevator, ElevatorConstants.CORAL_INTAKE_HEIGHT));

        NamedCommands.registerCommand("PrescoreCoral", new EndEffectorSetAngle(endEffector, EndEffectorConstants.PRESCORING_ANGLE));
        NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(endEffector, elevator, flywheel));

        NamedCommands.registerCommand("FeedCoral", new FeedCoral(flywheel));

        AutoManager.configureAutos();
    }

    /**
     * This method will be used to configure controls
     */
    private void configureBindings() {
        // commandGenericHID.button(1).onTrue(flywheel.setVelocity(1000));
        // commandGenericHID.button(1).onFalse(flywheel.setVelocity(0));

        commandGenericHID.button(1).onTrue(Commands.runOnce(() -> elevator.setSetpoint(ElevatorConstants.MIN_HEIGHT)));
        commandGenericHID.povDown()
                .onTrue(Commands.runOnce(() -> elevator.setSetpoint(FieldConstants.CORAL_LEVEL_HEIGHTS[0])));
        commandGenericHID.povUp()
                .onTrue(Commands.runOnce(() -> elevator.setSetpoint(FieldConstants.CORAL_LEVEL_HEIGHTS[1])));
        commandGenericHID.button(5)
                .onTrue(Commands.runOnce(() -> elevator.setSetpoint(FieldConstants.CORAL_LEVEL_HEIGHTS[2])));
        commandGenericHID.axisGreaterThan(2, 0.4)
                .onTrue(Commands.runOnce(() -> elevator.setSetpoint(FieldConstants.CORAL_LEVEL_HEIGHTS[3])));

        commandGenericHID.povLeft().onTrue(new AutoScoreCoral(drive, elevator, vision, true));
        commandGenericHID.povRight().onTrue(new AutoScoreCoral(drive, elevator, vision, false));

        commandGenericHID.button(2).onTrue(new AutoFeedCoral(drive, false));
        commandGenericHID.button(3).onTrue(new AutoFeedCoral(drive, true));

        commandGenericHID.button(4).onTrue(new EndEffectorSetAngle(endEffector, 0));
        commandGenericHID.button(4).onFalse(new EndEffectorSetAngle(endEffector, 120));

        // commandGenericHID.button(4).onTrue(new ScoreCoral(endEffector, elevator,
        // flywheel));

        /*
         * Set the drive subsystem to use the command returned by getTeleopCommand
         * if no other command is scheduled for the subsystem
         */
        drive.setDefaultCommand(drive.getTeleopCommand(elevator, commandGenericHID));
    }
}
