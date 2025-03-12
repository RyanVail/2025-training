package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    EndEffectorIO io;
    PIDController pid;
    State lastStateSetpoint;
    State stateSetpoint;
    double setpoint;
    MechanismLigament2d realLigament;
    MechanismLigament2d setpointLigament;

    private static final String LPREFIX = "/Subsystems/EndEffector/";

    public EndEffector(EndEffectorIO io, MechanismLigament2d realElevatorMech,
            MechanismLigament2d setpointElevatorMech) {
        lastStateSetpoint = new State();
        stateSetpoint = new State();

        this.io = io;
        this.pid = new PIDController(
                EndEffectorConstants.P,
                EndEffectorConstants.I,
                EndEffectorConstants.D);

        realLigament = realElevatorMech.append(
                new MechanismLigament2d(
                        "realEffector",
                        EndEffectorConstants.LENGTH,
                        0));

        realLigament.setLineWeight(5);
        realLigament.setColor(new Color8Bit(0, 0, 255));

        setpointLigament = setpointElevatorMech.append(
                new MechanismLigament2d(
                        "EffectorSetpoint",
                        EndEffectorConstants.LENGTH,
                        0));

        setpointLigament.setLineWeight(2);
        setpointLigament.setColor(new Color8Bit(0, 255, 0));

        SmartDashboard.putData("EndEffectorPID", pid);
    }

    @Override
    public void periodic() {
        lastStateSetpoint = EndEffectorConstants.PROFILE.calculate(0.02, lastStateSetpoint, stateSetpoint);

        double volts = this.pid.calculate(getAngle(), lastStateSetpoint.position);
        volts = Math.min(volts, RobotController.getBatteryVoltage());
        volts = Math.max(volts, -RobotController.getBatteryVoltage());
        io.setVoltage(volts);

        realLigament.setAngle(EndEffectorConstants.VISUALIZATION_BASE_ANGLE + getAngle());

        Logger.recordOutput(LPREFIX + "Setpoint", setpoint);
        Logger.recordOutput(LPREFIX + "Volts", volts);
        Logger.recordOutput(LPREFIX + "Angle", getAngle());

        this.io.periodic();
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void setSetpoint(double angle) {
        setpoint = angle;
        stateSetpoint = new State(angle, 0.0);
        setpointLigament.setAngle(EndEffectorConstants.VISUALIZATION_BASE_ANGLE + angle);
    }

    public double getAngle() {
        return io.getAngle();
    }

    public void zeroEncoders() {
        io.zeroEncoders();
    }
}