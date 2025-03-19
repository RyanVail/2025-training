package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.BeaterBarConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.commands.AlignFeed;
import frc.robot.commands.AlignIntakeAlgae;
import frc.robot.commands.AlignPose;
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.AlignScoreCoral;
import frc.robot.commands.BeaterBarSpin;
import frc.robot.commands.ElevatorSetHeight;
import frc.robot.commands.EndEffectorSetAngle;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.SlowStop;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.WaitController;
import frc.robot.commands.AlignPose.AlignCamera;
import frc.robot.commands.AlignScoreCoral.Side;
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
        elevator.zeroEncoders();

        VisionManager.initialize(drive);
        VisionManager.defaultCameras();

        configureAuto();

        configureBindings();
        drive.resetGyroOffset();
    }

    private void configureAuto() {
        NamedCommands.registerCommand(
                "StopDrive",
                Commands.runOnce(() -> drive.stop()));

        NamedCommands.registerCommand(
                "KeepStopDrive",
                Commands.run(() -> drive.stop()));

        NamedCommands.registerCommand(
                "SlowStop",
                new SlowStop(drive));

        NamedCommands.registerCommand(
                "ElevatorL4",
                new ElevatorSetHeight(elevator, ElevatorConstants.CORAL_LEVEL_HEIGHTS[3]));

        NamedCommands.registerCommand(
                "AlignLeft",
                new AlignScoreCoral(drive, Side.LEFT));

        NamedCommands.registerCommand(
                "AlignRight",
                new AlignScoreCoral(drive, Side.RIGHT));

        NamedCommands.registerCommand(
                "EffectorDown",
                new EndEffectorSetAngle(endEffector, elevator, EndEffectorConstants.HIGH_SCORE_ANGLE));

        NamedCommands.registerCommand(
                "IntakeCoral",
                Commands.sequence(
                        Commands.parallel(
                                new ElevatorSetHeight(elevator,
                                        ElevatorConstants.CORAL_INTAKE_HEIGHT),
                                (new EndEffectorSetAngle(endEffector, elevator,
                                        EndEffectorConstants.INTAKE_ANGLE))),
                        new IntakeCoral(intake)));

        NamedCommands.registerCommand(
                "IdleCoral",
                Commands.sequence(new ElevatorSetHeight(elevator,
                        ElevatorConstants.CORAL_LEVEL_HEIGHTS[1]),
                        new EndEffectorSetAngle(endEffector, elevator,
                                EndEffectorConstants.IDLE_ANGLE)));

        NamedCommands.registerCommand("AlignFeeder", new AlignFeed(drive));

        NamedCommands.registerCommand(
                "EjectCoral",
                Commands.sequence(
                        new EjectCoral(intake)));

        AutoManager.configureAutos(drive, elevator, endEffector);
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
                Commands.parallel(
                        new ElevatorSetHeight(elevator,
                                ElevatorConstants.CORAL_LEVEL_HEIGHTS[1]),
                        new EndEffectorSetAngle(endEffector, elevator,
                                EndEffectorConstants.IDLE_ANGLE)));

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
                Commands.parallel(
                        new ElevatorSetHeight(elevator,
                                ElevatorConstants.CORAL_LEVEL_HEIGHTS[1]),
                        new EndEffectorSetAngle(endEffector, elevator,
                                EndEffectorConstants.IDLE_ANGLE)));

        c.setName("EjectAndResetCommand");

        Command eject_cancel = Commands.none();
        eject_cancel.addRequirements(drive);

        operatorHID.button(XboxController.Button.kY.value).onTrue(eject_cancel);
        operatorHID.button(XboxController.Button.kY.value).onTrue(c);

        operatorHID.button(XboxController.Button.kBack.value)
                .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

        if (Robot.isReal())
            driverHID.button(XboxController.Button.kY.value).onTrue(new EjectAlgae(intake));

        operatorHID.povLeft().onTrue(new AlignScoreCoral(drive, Side.LEFT));
        operatorHID.povRight().onTrue(new AlignScoreCoral(drive, Side.RIGHT));

        if (Robot.isReal()) {
            driverHID.axisGreaterThan(XboxController.Axis.kLeftTrigger.value,
                    InputConstants.TRIGGER_THRESHOLD).onTrue(
                            Commands.runOnce(() -> beaterBar
                                    .setSpeed(BeaterBarConstants.INTAKE_SPEED)));
            driverHID.axisGreaterThan(XboxController.Axis.kLeftTrigger.value,
                    InputConstants.TRIGGER_THRESHOLD)
                    .onFalse(Commands.runOnce(() -> beaterBar.setSpeed(0)));

            driverHID.axisGreaterThan(XboxController.Axis.kRightTrigger.value,
                    InputConstants.TRIGGER_THRESHOLD).onTrue(
                            Commands.runOnce(() -> beaterBar
                                    .setSpeed(BeaterBarConstants.EJECT_SPEED)));
            driverHID.axisGreaterThan(XboxController.Axis.kRightTrigger.value,
                    InputConstants.TRIGGER_THRESHOLD)
                    .onFalse(Commands.runOnce(() -> beaterBar.setSpeed(0)));
        }

        operatorHID.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, InputConstants.TRIGGER_THRESHOLD)
                .onTrue(
                        Commands.sequence(
                                Commands.parallel(
                                        new ElevatorSetHeight(elevator,
                                                ElevatorConstants.BARGE_HEIGHT),
                                        new EndEffectorSetAngle(endEffector,
                                                elevator,
                                                EndEffectorConstants.ALGAE_BARGE_EJECT_ANGLE))));

        operatorHID.button(XboxController.Button.kA.value)
                .onTrue(new EndEffectorSetAngle(endEffector, elevator,
                        EndEffectorConstants.LOW_SCORE_ANGLE));

        Command feeder_align = Commands.race(
                new AlignFeed(drive),
                new BeaterBarSpin(beaterBar, BeaterBarConstants.FEEDER_SPEED));

        if (Robot.isReal())
            feeder_align = feeder_align
                    .raceWith(new WaitController(driverHID, XboxController.Button.kRightBumper));

        operatorHID.button(XboxController.Button.kB.value).onTrue(feeder_align);

        operatorHID.button(XboxController.Button.kStart.value)
                .onTrue(new ElevatorSetHeight(elevator, ElevatorConstants.PROCESSOR_HEIGHT)
                        .andThen(new EndEffectorSetAngle(endEffector, elevator,
                                EndEffectorConstants.PROCESSOR_ANGLE)));

        AlignIntakeAlgae high_algae_intake = new AlignIntakeAlgae(drive);
        operatorHID.axisMagnitudeGreaterThan(XboxController.Axis.kRightTrigger.value,
                InputConstants.TRIGGER_THRESHOLD)
                .onTrue(
                        Commands.either(
                                Commands.deadline(
                                        Commands.parallel(
                                                new ElevatorSetHeight(
                                                        elevator,
                                                        ElevatorConstants.ALGAE_LEVEL_HEIGHTS[1]),
                                                new EndEffectorSetAngle(
                                                        endEffector,
                                                        elevator,
                                                        EndEffectorConstants.ALGAE_INTAKE_ANGLE)),
                                        new SlowStop(drive))
                                        .andThen(
                                                Commands.deadline(
                                                        new IntakeAlgae(intake),
                                                        high_algae_intake)),
                                Commands.none(),
                                () -> {
                                    high_algae_intake.findTargetPose();
                                    return high_algae_intake.canRun();
                                }));

        AlignIntakeAlgae low_algae_intake = new AlignIntakeAlgae(drive);
        operatorHID.button(XboxController.Button.kRightBumper.value)
                .onTrue(
                        Commands.either(
                                Commands.deadline(
                                        Commands.parallel(
                                                new ElevatorSetHeight(
                                                        elevator,
                                                        ElevatorConstants.ALGAE_LEVEL_HEIGHTS[0]),
                                                new EndEffectorSetAngle(
                                                        endEffector,
                                                        elevator,
                                                        EndEffectorConstants.ALGAE_INTAKE_ANGLE)),
                                        new SlowStop(drive))
                                        .andThen(
                                                Commands.deadline(
                                                        new IntakeAlgae(intake),
                                                        low_algae_intake)),
                                Commands.none(),
                                () -> {
                                    low_algae_intake.findTargetPose();
                                    return low_algae_intake.canRun();
                                }));

        drive.setDefaultCommand(
                Commands.either(
                        drive.getTeleopCommand(elevator,
                                (Robot.isReal()) ? driverHID : operatorHID),
                        new SlowStop(drive),
                        () -> DriverStation.isTeleop()));
    }
}
