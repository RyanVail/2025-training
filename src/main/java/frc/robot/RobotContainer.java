package frc.robot;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AlignPose;
import frc.robot.commands.AlignIntakeAlgae;
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.AlignScoreCoral;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.ElevatorSetHeight;
import frc.robot.commands.EndEffectorSetAngle;
import frc.robot.commands.FeedCoral;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.WaitController;
import frc.robot.commands.AlignPose.AlignCamera;
import frc.robot.subsystems.beaterbar.BeaterBar;
import frc.robot.subsystems.beaterbar.BeaterBarIOFlex;
import frc.robot.subsystems.beaterbar.BeaterBarIOSim;
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
import frc.robot.subsystems.vision.VisionManager;

public class RobotContainer {
        CommandGenericHID commandGenericHID;
        CommandGenericHID driverHID;

        Drive drive;
        Intake intake;
        Elevator elevator;
        EndEffector endEffector;
        BeaterBar beaterBar;

        public RobotContainer() {
                commandGenericHID = new CommandGenericHID(Constants.CONTROLLER_PORT);
                driverHID = new CommandGenericHID(1); // TODO: Make a constants somewhere.

                if (Robot.isSimulation()) {
                        drive = new Drive(new DriveIOSwerve());
                        intake = new Intake(new IntakeIOSim());
                        elevator = new Elevator(new ElevatorIOSim());
                        endEffector = new EndEffector(new EndEffectorIOSim(), elevator.getRealMech(),
                                        elevator.getSetpointMech());
                        beaterBar = new BeaterBar(new BeaterBarIOSim());
                } else {
                        drive = new Drive(new DriveIOSwerve());
                        intake = new Intake(new IntakeIOSpark());
                        elevator = new Elevator(new ElevatorIOSpark());
                        endEffector = new EndEffector(new EndEffectorIOSpark(), elevator.getRealMech(),
                                        elevator.getSetpointMech());
                        beaterBar = new BeaterBar(new BeaterBarIOFlex());
                }

                endEffector.zeroEncoders();

                VisionManager.initialize();

                configureBindings();
                configureAuto();
                drive.resetGyroOffset();
        }

        private void configureAuto() {
                // NamedCommands.registerCommand("ElevatorL1", new ElevatorSetHeight(elevator,
                // ElevatorConstants.CORAL_SCORE_OFFSET +
                // FieldConstants.CORAL_LEVEL_HEIGHTS[0]));
                // NamedCommands.registerCommand("ElevatorL2", new ElevatorSetHeight(elevator,
                // ElevatorConstants.CORAL_SCORE_OFFSET +
                // FieldConstants.CORAL_LEVEL_HEIGHTS[1]));
                // NamedCommands.registerCommand("ElevatorL3", new ElevatorSetHeight(elevator,
                // ElevatorConstants.CORAL_SCORE_OFFSET +
                // FieldConstants.CORAL_LEVEL_HEIGHTS[2]));
                // NamedCommands.registerCommand("ElevatorL4", new ElevatorSetHeight(elevator,
                // ElevatorConstants.CORAL_SCORE_OFFSET +
                // FieldConstants.CORAL_LEVEL_HEIGHTS[3]));
                // NamedCommands.registerCommand("ElevatorIntake",
                // new ElevatorSetHeight(elevator, ElevatorConstants.CORAL_INTAKE_HEIGHT));

                // NamedCommands.registerCommand("PrescoreCoral",
                // new EndEffectorSetAngle(endEffector, EndEffectorConstants.PRESCORING_ANGLE));
                // NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(endEffector,
                // elevator, intake));

                // NamedCommands.registerCommand("FeedCoral", new FeedCoral(intake));

                // AutoManager.configureAutos();
        }

        /**
         * This method will be used to configure controls
         */
        private void configureBindings() {
                SmartDashboard.putData("Zero Encoders", Commands.runOnce(() -> {
                        elevator.zeroEncoders();
                        endEffector.zeroEncoders();
                }));

                SmartDashboard.putData("Reset Gyro", Commands.runOnce(() -> {
                        drive.resetGyroOffset();
                }));

                driverHID.button(XboxController.Button.kStart.value).onTrue(Commands.runOnce(() -> {
                        drive.resetGyroOffset();
                }));

                SmartDashboard.putBoolean("Command Verbose Logging", false);

                CommandScheduler.getInstance().onCommandInitialize((Command c) -> {
                        if (!(c instanceof PrintCommand)
                                        && SmartDashboard.getBoolean("Command Verbose Logging", false)) {
                                Commands.print(c.getName() + " initialized").schedule();
                        }
                });

                CommandScheduler.getInstance().onCommandFinish((Command c) -> {
                        if (!(c instanceof PrintCommand)
                                        && SmartDashboard.getBoolean("Command Verbose Logging", false)) {
                                Commands.print(c.getName() + " initialized").schedule();
                        }
                });

                CommandScheduler.getInstance().onCommandInterrupt((Command c1, Optional<Command> c2) -> {
                        if (c2.isPresent() && SmartDashboard.getBoolean("Command Verbose Logging",
                                        false))
                                Commands.print(c1.getName() + " interrupted by " + c2.get().getName()).schedule();
                        ;
                });

                Command intake_command = Commands.sequence(
                                Commands.parallel(
                                                new ElevatorSetHeight(elevator, ElevatorConstants.CORAL_INTAKE_HEIGHT),
                                                (new EndEffectorSetAngle(endEffector, elevator,
                                                                EndEffectorConstants.INTAKE_ANGLE))),
                                new FeedCoral(intake));

                commandGenericHID.button(XboxController.Button.kX.value).onTrue(intake_command);

                commandGenericHID.povDown().onTrue(
                                new ElevatorSetHeight(elevator, FieldConstants.CORAL_LEVEL_HEIGHTS[1])
                                                .andThen(new EndEffectorSetAngle(endEffector, elevator,
                                                                EndEffectorConstants.SCORING_ANGLES[0])));

                commandGenericHID.povUp().onTrue(
                                new ElevatorSetHeight(elevator, FieldConstants.CORAL_LEVEL_HEIGHTS[2])
                                                .andThen(new EndEffectorSetAngle(endEffector, elevator,
                                                                EndEffectorConstants.SCORING_ANGLES[0])));

                commandGenericHID.button(XboxController.Button.kRightBumper.value).onTrue(
                                new ElevatorSetHeight(elevator, FieldConstants.CORAL_LEVEL_HEIGHTS[3])
                                                .andThen(new EndEffectorSetAngle(endEffector, elevator,
                                                                EndEffectorConstants.SCORING_ANGLES[2])));

                // commandGenericHID.povDown().onTrue(
                // new ElevatorSetHeight(elevator, FieldConstants.CORAL_LEVEL_HEIGHTS[2])
                // .andThen(new EndEffectorSetAngle(endEffector, elevator,
                // EndEffectorConstants.SCORING_ANGLES[0])));

                commandGenericHID.button(XboxController.Button.kY.value).onTrue(new EjectCoral(intake));

                driverHID.button(XboxController.Button.kY.value).onTrue(new EjectAlgae(intake));

                commandGenericHID.povLeft().onTrue(new AlignScoreCoral(drive, true));
                commandGenericHID.povRight().onTrue(new AlignScoreCoral(drive, false));

                commandGenericHID.button(XboxController.Button.kBack.value)
                                .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

                // commandGenericHID.povLeft()
                // .onTrue(Commands.runOnce(() ->
                // beaterBar.setSpeed(BeaterBarConstants.FEED_SPEED)));
                // commandGenericHID.povLeft().onFalse(Commands.runOnce(() ->
                // beaterBar.setSpeed(0)));
                // commandGenericHID.povRight()
                // .onTrue(Commands.runOnce(() ->
                // beaterBar.setSpeed(-BeaterBarConstants.FEED_SPEED * 3)));
                // commandGenericHID.povRight().onFalse(Commands.runOnce(() ->
                // beaterBar.setSpeed(0)));

                commandGenericHID.button(XboxController.Button.kA.value)
                                .onTrue(new EndEffectorSetAngle(endEffector, elevator,
                                                EndEffectorConstants.SCORING_ANGLES[0]));

                commandGenericHID.button(XboxController.Button.kB.value)
                                .onTrue(
                                                Commands.race(
                                                                new WaitController(driverHID, XboxController.Button.kA),
                                                                new AlignPose(drive,
                                                                                List.of(FlippingUtil.flipFieldPose(
                                                                                                FieldConstants.FEEDER_POSES[0])),
                                                                                AlignCamera.Back)));

                commandGenericHID.button(XboxController.Button.kStart.value)
                                .onTrue(new ElevatorSetHeight(elevator, FieldConstants.PROCESSOR_HEIGHT)
                                                .andThen(new EndEffectorSetAngle(endEffector, elevator,
                                                                EndEffectorConstants.PROCESSOR_ANGLE)));

                // TODO: Maybe these shouldn't run if they're out of the range.
                commandGenericHID.axisMagnitudeGreaterThan(XboxController.Axis.kRightTrigger.value,
                                0.6)
                                .onTrue(
                                                Commands.parallel(
                                                                new ElevatorSetHeight(elevator,
                                                                                FieldConstants.ALGEA_LEVEL_HEIGHTS[1]),
                                                                new EndEffectorSetAngle(endEffector, elevator,
                                                                                EndEffectorConstants.ALGAE_INTAKE_ANGLE))
                                                                .andThen(
                                                                                Commands.deadline(
                                                                                                new IntakeAlgae(intake),
                                                                                                new AlignIntakeAlgae(
                                                                                                                drive))));

                commandGenericHID.button(XboxController.Button.kRightBumper.value)
                                .onTrue(
                                                Commands.parallel(
                                                                new ElevatorSetHeight(elevator,
                                                                                FieldConstants.ALGEA_LEVEL_HEIGHTS[0]),
                                                                new EndEffectorSetAngle(endEffector, elevator,
                                                                                EndEffectorConstants.ALGAE_INTAKE_ANGLE))
                                                                .andThen(
                                                                                Commands.deadline(
                                                                                                new IntakeAlgae(intake),
                                                                                                new AlignIntakeAlgae(
                                                                                                                drive))));

                /*
                 * Set the drive subsystem to use the command returned by getTeleopCommand
                 * if no other command is scheduled for the subsystem
                 */
                drive.setDefaultCommand(drive.getTeleopCommand(elevator, driverHID));
        }
}
