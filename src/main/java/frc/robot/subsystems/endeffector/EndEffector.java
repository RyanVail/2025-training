package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    EndEffectorIO io;
    PIDController pid;
    double setpoint;
    MechanismLigament2d realLigament;
    MechanismLigament2d setpointLigament;

    private static final String LPREFIX = "/Subsystems/EndEffector/";

    public EndEffector(EndEffectorIO io, MechanismLigament2d realElevatorMech,
            MechanismLigament2d setpointElevatorMech) {
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
    }

    @Override
    public void periodic() {
        double volts = this.pid.calculate(getAngle());
        volts = Math.min(volts, RobotController.getBatteryVoltage());
        volts = Math.max(volts, -RobotController.getBatteryVoltage());
        io.setVoltage(volts);

        realLigament.setAngle(EndEffectorConstants.VISUALIZATION_BASE_ANGLE + getAngle());

        SmartDashboard.putData(LPREFIX + "PID", pid);

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
        pid.setSetpoint(angle);
        setpointLigament.setAngle(EndEffectorConstants.VISUALIZATION_BASE_ANGLE + angle);
    }

    public double getAngle() {
        return io.getAngle() * 10;
    }
}