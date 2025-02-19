package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.BeaterBarConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AutoFeedCoral;
import frc.robot.commands.AutoScoreCoral;
import frc.robot.commands.ElevatorSetHeight;
import frc.robot.commands.EndEffectorSetAngle;
import frc.robot.commands.FeedCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.beaterbar.BeaterBarIOFlex;
import frc.robot.subsystems.beaterbar.BeaterBarIOSim;
import frc.robot.subsystems.beaterbar.BeaterBar;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSwerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOSim;

public class RobotContainer {
        CommandGenericHID commandGenericHID;

        Drive drive;
        Intake intake;
        Elevator elevator;
        Vision vision;
        EndEffector endEffector;
        BeaterBar beaterBar;

        public RobotContainer() {
                commandGenericHID = new CommandGenericHID(Constants.CONTROLLER_PORT);

                if (Robot.isSimulation()) {
                        drive = new Drive(new DriveIOSwerve());
                        intake = new Intake(new IntakeIOSim());
                        elevator = new Elevator(new ElevatorIOSim());
                        vision = new Vision(new VisionIOSim(), drive.getPoseSupplier());
                        endEffector = new EndEffector(new EndEffectorIOSim(), elevator.getRealMech(),
                                        elevator.getSetpointMech());
                        beaterBar = new BeaterBar(new BeaterBarIOSim());
                } else {
                        // drive = new Drive(new DriveIOSwerve());
                        intake = new Intake(new IntakeIOSpark());
                        elevator = new Elevator(new ElevatorIOSpark());
                        // TODO: Vision real.
                        endEffector = new EndEffector(new EndEffectorIOSpark(), elevator.getRealMech(),
                                        elevator.getSetpointMech());
                        // beaterBar = new BeaterBar(new BeaterBarIOFlex());
                }

                configureBindings();
                configureAuto();
        }

        private void configureAuto() {
                // NamedCommands.registerCommand("ElevatorL1", new ElevatorSetHeight(elevator,
                //                 ElevatorConstants.CORAL_SCORE_OFFSET + FieldConstants.CORAL_LEVEL_HEIGHTS[0]));
                // NamedCommands.registerCommand("ElevatorL2", new ElevatorSetHeight(elevator,
                //                 ElevatorConstants.CORAL_SCORE_OFFSET + FieldConstants.CORAL_LEVEL_HEIGHTS[1]));
                // NamedCommands.registerCommand("ElevatorL3", new ElevatorSetHeight(elevator,
                //                 ElevatorConstants.CORAL_SCORE_OFFSET + FieldConstants.CORAL_LEVEL_HEIGHTS[2]));
                // NamedCommands.registerCommand("ElevatorL4", new ElevatorSetHeight(elevator,
                //                 ElevatorConstants.CORAL_SCORE_OFFSET + FieldConstants.CORAL_LEVEL_HEIGHTS[3]));
                // NamedCommands.registerCommand("ElevatorIntake",
                //                 new ElevatorSetHeight(elevator, ElevatorConstants.CORAL_INTAKE_HEIGHT));

                // NamedCommands.registerCommand("PrescoreCoral",
                //                 new EndEffectorSetAngle(endEffector, EndEffectorConstants.PRESCORING_ANGLE));
                // NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(endEffector, elevator, intake));

                // NamedCommands.registerCommand("FeedCoral", new FeedCoral(intake));

                // AutoManager.configureAutos();
        }

        /**
         * This method will be used to configure controls
         */
        private void configureBindings() {
                
                commandGenericHID.button(3).onTrue(Commands.runOnce(() -> intake.setVoltage(IntakeConstants.CORAL_FEED_VOLTAGE)));
                commandGenericHID.button(3).onFalse(Commands.runOnce(() -> intake.setVoltage(0)));

                commandGenericHID.button(4).onTrue(Commands.runOnce(() -> intake.setVoltage(IntakeConstants.CORAL_SCORE_VOLTAGE)));
                commandGenericHID.button(4).onFalse(Commands.runOnce(() -> intake.setVoltage(0)));

                // commandGenericHID.button(1)
                //                 .onTrue(Commands.runOnce(() -> elevator.setSetpoint(ElevatorConstants.MIN_HEIGHT)));
                commandGenericHID.povDown()
                                .onTrue(Commands.runOnce(
                                                () -> elevator.setSetpoint(ElevatorConstants.CORAL_INTAKE_HEIGHT)));
                // commandGenericHID.povUp()
                //                 .onTrue(Commands.runOnce(
                //                                 () -> elevator.setSetpoint(FieldConstants.CORAL_LEVEL_HEIGHTS[1])));
                // commandGenericHID.button(5)
                //                 .onTrue(Commands.runOnce(
                //                                 () -> elevator.setSetpoint(FieldConstants.CORAL_LEVEL_HEIGHTS[2])));
                // commandGenericHID.axisGreaterThan(2, 0.4)
                //                 .onTrue(Commands.runOnce(
                //                                 () -> elevator.setSetpoint(FieldConstants.CORAL_LEVEL_HEIGHTS[3])));

                // commandGenericHID.povLeft().onTrue(new AutoScoreCoral(drive, elevator, vision, true));
                // commandGenericHID.povRight().onTrue(new AutoScoreCoral(drive, elevator, vision, false));

                // commandGenericHID.button(2).onTrue(new AutoFeedCoral(drive, false));
                // commandGenericHID.button(3).onTrue(new AutoFeedCoral(drive, true));

                // // TODO: This is only for debugging.
                commandGenericHID.button(1).onTrue(new EndEffectorSetAngle(endEffector, EndEffectorConstants.SCORING_ANGLES[0]));
                // commandGenericHID.button(2).onTrue(new EndEffectorSetAngle(endEffector, EndEffectorConstants.SCORING_ANGLES[1]));

                // commandGenericHID.button(2).onTrue(new EndEffectorSetAngle(endEffector, EndEffectorConstants.SCORING_ANGLES[1]));
                commandGenericHID.button(2).onTrue(new EndEffectorSetAngle(endEffector, EndEffectorConstants.INTAKE_ANGLE));

                // commandGenericHID.button(6).toggleOnTrue(
                //                 Commands.runOnce(() -> beaterBar.setSpeed(BeaterBarConstants.FEED_SPEED)));
                // commandGenericHID.button(6).toggleOnFalse(Commands.runOnce(() -> beaterBar.setSpeed(0)));

                // commandGenericHID.button(4).onTrue(new ScoreCoral(endEffector, elevator,
                // intake));

                /*
                 * Set the drive subsystem to use the command returned by getTeleopCommand
                 * if no other command is scheduled for the subsystem
                 */
                // drive.setDefaultCommand(drive.getTeleopCommand(elevator, commandGenericHID));
        }
}
