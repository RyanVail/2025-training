package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    ElevatorSubsystemIO io;
    PIDController pid;
    ElevatorFeedforward feedForward;

    Mechanism2d mechanism;
    MechanismLigament2d realLigament;
    MechanismLigament2d setpointLigament;

    private static final String LPREFIX = "/Subsystems/Elevator/";

    public ElevatorSubsystem(ElevatorSubsystemIO io) {
        this.io = io;
        this.pid = new PIDController(
                Constants.ElevatorConstants.P,
                Constants.ElevatorConstants.I,
                Constants.ElevatorConstants.D);

        this.feedForward = new ElevatorFeedforward(
                Constants.ElevatorConstants.S,
                Constants.ElevatorConstants.G,
                Constants.ElevatorConstants.V);

        this.mechanism = new Mechanism2d(0, 0);
        MechanismRoot2d root = mechanism.getRoot(
                "elevator",
                Constants.ElevatorConstants.POS_X,
                Constants.ElevatorConstants.POS_Y);

        realLigament = root.append(
                new MechanismLigament2d("realElevator",
                        Constants.ElevatorConstants.HEIGHT,
                        90));

        realLigament.setLineWeight(5);
        realLigament.setColor(new Color8Bit(0, 0, 255));

        setpointLigament = root.append(
                new MechanismLigament2d("elevatorSetpoint",
                        Constants.ElevatorConstants.HEIGHT,
                        90));

        setpointLigament.setLineWeight(2);
        setpointLigament.setColor(new Color8Bit(0, 255, 0));
    }

    @Override
    public void periodic() {
        io.updateSimulation();

        double height = io.getHeight();
        double voltage = pid.calculate(height);
        voltage += feedForward.calculate(height);
        io.setVoltage(voltage);

        SmartDashboard.putData(LPREFIX + "PID", pid);

        SmartDashboard.putNumber(LPREFIX + "Voltage", voltage);
        realLigament.setLength(height);
        SmartDashboard.putData(LPREFIX + "Mech2D", mechanism);
    }

    public Command setHeight(double setpoint) {
        return Commands.runOnce(() -> {
            pid.setSetpoint(setpoint);
            setpointLigament.setLength(setpoint);
        });
    }
}
