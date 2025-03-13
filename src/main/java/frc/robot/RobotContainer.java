package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.commands.AlignPose;
import frc.robot.commands.AlignFeed;
import frc.robot.commands.AlignIntakeAlgae;
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.AlignScoreCoral;
import frc.robot.commands.ElevatorSetHeight;
import frc.robot.commands.EndEffectorSetAngle;
import frc.robot.commands.IntakeCoral;
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
        CommandGenericHID operatorHID;
        CommandGenericHID driverHID;

        Drive drive;
        Intake intake;
        Elevator elevator;
        EndEffector endEffector;
        BeaterBar beaterBar;

        public RobotContainer() {
                operatorHID = new CommandGenericHID(InputConstants.OPERATOR_CONTROLLER_PORT);

                // For simulations it's easier to have one controller.
                if (Robot.isReal())
                        driverHID = new CommandGenericHID(InputConstants.DRIVE_CONTROLLER_PORT);

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

                SmartDashboard.putData("Command scheduler", CommandScheduler.getInstance());

                if (Robot.isReal()) {
                        driverHID.button(XboxController.Button.kStart.value).onTrue(Commands.runOnce(() -> {
                                drive.resetGyroOffset();
                        }));
                }

                SmartDashboard.putBoolean("Command Verbose Logging", false);

                // CommandScheduler.getInstance().onCommandInitialize((Command c) -> {
                // if (!(c instanceof PrintCommand)
                // && SmartDashboard.getBoolean("Command Verbose Logging", false)) {
                // Commands.print(c.getName() + " initialized").schedule();
                // }
                // });

                // CommandScheduler.getInstance().onCommandFinish((Command c) -> {
                // if (!(c instanceof PrintCommand)
                // && SmartDashboard.getBoolean("Command Verbose Logging", false)) {
                // Commands.print(c.getName() + " initialized").schedule();
                // }
                // });

                // CommandScheduler.getInstance().onCommandInterrupt((Command c1,
                // Optional<Command> c2) -> {
                // if (c2.isPresent() && SmartDashboard.getBoolean("Command Verbose Logging",
                // false))
                // Commands.print(c1.getName() + " interrupted by " +
                // c2.get().getName()).schedule();
                // ;
                // });

                Command intake_command = Commands.sequence(
                                Commands.parallel(
                                                new ElevatorSetHeight(elevator, ElevatorConstants.CORAL_INTAKE_HEIGHT),
                                                (new EndEffectorSetAngle(endEffector, elevator,
                                                                EndEffectorConstants.INTAKE_ANGLE))),
                                new IntakeCoral(intake),
                                new ElevatorSetHeight(elevator, ElevatorConstants.CORAL_LEVEL_HEIGHTS[1]),
                                new EndEffectorSetAngle(endEffector, elevator, EndEffectorConstants.IDLE_ANGLE));

                operatorHID.button(XboxController.Button.kX.value).onTrue(intake_command);
                operatorHID.button(XboxController.Button.kRightStick.value).onTrue(
                                new EndEffectorSetAngle(endEffector, elevator, EndEffectorConstants.IDLE_ANGLE));

                operatorHID.povDown().onTrue(
                                new ElevatorSetHeight(elevator, ElevatorConstants.CORAL_LEVEL_HEIGHTS[1])
                                                .andThen(new EndEffectorSetAngle(endEffector, elevator,
                                                                EndEffectorConstants.LOW_SCORE_ANGLE)));

                operatorHID.povUp().onTrue(
                                new ElevatorSetHeight(elevator, ElevatorConstants.CORAL_LEVEL_HEIGHTS[2])
                                                .andThen(new EndEffectorSetAngle(endEffector, elevator,
                                                                EndEffectorConstants.LOW_SCORE_ANGLE)));

                operatorHID.button(XboxController.Button.kLeftBumper.value).onTrue(
                                new ElevatorSetHeight(elevator, ElevatorConstants.CORAL_LEVEL_HEIGHTS[3])
                                                .andThen(new EndEffectorSetAngle(endEffector, elevator,
                                                                EndEffectorConstants.HIGH_SCORE_ANGLE)));

                Command c = Commands.sequence(
                                new EjectCoral(intake),
                                // new CoralScoreReset(drive),
                                Commands.parallel(
                                                new ElevatorSetHeight(elevator,
                                                                ElevatorConstants.CORAL_LEVEL_HEIGHTS[1]),
                                                new EndEffectorSetAngle(endEffector, elevator,
                                                                EndEffectorConstants.IDLE_ANGLE)));

                c.setName("EjectAndResetCommand");

                operatorHID.button(XboxController.Button.kY.value)
                                .onTrue(c);

                if (Robot.isReal())
                        driverHID.button(XboxController.Button.kY.value).onTrue(new EjectAlgae(intake));

                operatorHID.povLeft().onTrue(new AlignScoreCoral(drive, true));
                operatorHID.povRight().onTrue(new AlignScoreCoral(drive, false));

                operatorHID.button(XboxController.Button.kBack.value).onTrue(new AlignFeed(drive));

                // commandGenericHID.povLeft()
                // .onTrue(Commands.runOnce(() ->
                // beaterBar.setSpeed(BeaterBarConstants.INTAKE_SPEED)));
                // commandGenericHID.povLeft().onFalse(Commands.runOnce(() ->
                // beaterBar.setSpeed(0)));

                // commandGenericHID.povRight()
                // .onTrue(Commands.runOnce(() ->
                // beaterBar.setSpeed(-BeaterBarConstants.EJECT_SPEED)));
                // commandGenericHID.povRight().onFalse(Commands.runOnce(() ->
                // beaterBar.setSpeed(0)));

                operatorHID.button(XboxController.Button.kA.value)
                                .onTrue(new EndEffectorSetAngle(endEffector, elevator,
                                                EndEffectorConstants.LOW_SCORE_ANGLE));

                Command feeder_align = new AlignPose(drive, FlippingUtil.flipFieldPose(
                                FieldConstants.FEEDER_POSES[0]),
                                AlignCamera.Back);

                if (Robot.isReal())
                        feeder_align = feeder_align.raceWith(new WaitController(driverHID, XboxController.Button.kA));

                operatorHID.button(XboxController.Button.kB.value).onTrue(feeder_align);

                operatorHID.button(XboxController.Button.kStart.value)
                                .onTrue(new ElevatorSetHeight(elevator, ElevatorConstants.PROCESSOR_HEIGHT)
                                                .andThen(new EndEffectorSetAngle(endEffector, elevator,
                                                                EndEffectorConstants.PROCESSOR_ANGLE)));

                // TODO: Maybe these shouldn't run if they're out of the range.
                operatorHID.axisMagnitudeGreaterThan(XboxController.Axis.kRightTrigger.value,
                                0.6)
                                .onTrue(
                                                Commands.parallel(
                                                                new ElevatorSetHeight(elevator,
                                                                                ElevatorConstants.ALGAE_LEVEL_HEIGHTS[1]),
                                                                new EndEffectorSetAngle(endEffector, elevator,
                                                                                EndEffectorConstants.ALGAE_INTAKE_ANGLE))
                                                                .andThen(
                                                                                Commands.deadline(
                                                                                                new IntakeAlgae(intake),
                                                                                                new AlignIntakeAlgae(drive))));

                operatorHID.button(XboxController.Button.kRightBumper.value)
                                .onTrue(
                                                Commands.parallel(
                                                                new ElevatorSetHeight(elevator,
                                                                                ElevatorConstants.ALGAE_LEVEL_HEIGHTS[0]),
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
                drive.setDefaultCommand(drive.getTeleopCommand(elevator, (Robot.isReal()) ? driverHID : operatorHID));
        }
}
