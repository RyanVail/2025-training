package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    ElevatorIO io;
    PIDController pid;
    ElevatorFeedforward feedForward;
    State lastStateSetpoint;
    State stateSetpoint;

    Mechanism2d mechanism;
    MechanismLigament2d realLigament;
    MechanismLigament2d setpointLigament;
    double setpoint;

    public static final String LPREFIX = "/Subsystems/Elevator/";

    public Elevator(ElevatorIO io) {
        lastStateSetpoint = new State();
        stateSetpoint = new State();
        this.io = io;
        this.pid = new PIDController(
                ElevatorConstants.P,
                ElevatorConstants.I,
                ElevatorConstants.D);

        this.feedForward = new ElevatorFeedforward(
                ElevatorConstants.S,
                ElevatorConstants.G,
                ElevatorConstants.V);

        this.mechanism = new Mechanism2d(0, 0);
        MechanismRoot2d root = mechanism.getRoot(
                "elevator",
                ElevatorConstants.POS.getX(),
                ElevatorConstants.POS.getY());

        realLigament = root.append(
                new MechanismLigament2d("realElevator",
                        ElevatorConstants.HEIGHT,
                        90));

        realLigament.setLineWeight(5);
        realLigament.setColor(new Color8Bit(0, 0, 255));

        setpointLigament = root.append(
                new MechanismLigament2d("elevatorSetpoint",
                        ElevatorConstants.HEIGHT,
                        90));

        setpointLigament.setLineWeight(2);
        setpointLigament.setColor(new Color8Bit(0, 255, 0));

        setpointLigament.setLength(0.0);

        SmartDashboard.putData("ElevatorPID", pid);
    }

    public MechanismLigament2d getRealMech() {
        return realLigament;
    }

    public MechanismLigament2d getSetpointMech() {
        return setpointLigament;
    }

    @Override
    public void periodic() {
        lastStateSetpoint = ElevatorConstants.PROFILE.calculate(0.02, lastStateSetpoint, stateSetpoint);

        double height = io.getHeight();
        double voltage = pid.calculate(height, lastStateSetpoint.position);
        voltage += feedForward.calculate(height); // TODO: Should this be passed the height?
        voltage = Math.min(voltage, RobotController.getBatteryVoltage());
        voltage = Math.max(voltage, -RobotController.getBatteryVoltage());
        io.setVoltage(voltage);

        realLigament.setLength(height * ElevatorConstants.VISUALIZATION_HEIGHT_MUL);
        SmartDashboard.putData(LPREFIX + "Mech2D", mechanism);

        Logger.recordOutput(LPREFIX + "Voltage", voltage);
        Logger.recordOutput(LPREFIX + "Height", height);
        Logger.recordOutput(LPREFIX + "Setpoint", setpoint);
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        stateSetpoint = new State(setpoint, 0.0);
        setpointLigament.setLength(setpoint * ElevatorConstants.VISUALIZATION_HEIGHT_MUL);
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getHeight() {
        return io.getHeight();
    }

    public void zeroEncoders() {
        io.zeroEncoders();
    }
}
