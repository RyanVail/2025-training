package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    private EndEffectorIO io;
    private PIDController pid;
    private double setpoint;

    public EndEffector(EndEffectorIO io) {
        this.io = io;
        this.pid = new PIDController(
            EndEffectorConstants.P,
            EndEffectorConstants.I,
            EndEffectorConstants.D
        );
    }

    @Override
    public void periodic() {
        double volts = this.pid.calculate(getAngle());
        volts = Math.min(volts, RobotController.getBatteryVoltage());
        volts = Math.max(volts, -RobotController.getBatteryVoltage());
        this.io.setVoltage(volts);

        SmartDashboard.putData("EndEffectorPID", pid);

        // TODO: Make this real.
        Logger.recordOutput("EndEffectorSetpoint", setpoint);
        Logger.recordOutput("EndEffectorVolts", volts);
        Logger.recordOutput("EndEffectorAngle", getAngle());
    }

    @Override
    public void simulationPeriodic() {
        this.io.simulationPeriodic();
    }

    public void setSetpoint(double angle) {
        this.setpoint = angle;
        this.pid.setSetpoint(angle);
    }

    // TODO: This doesn't work like this.
    public double getAngle() {
        return io.getAngle();
    }
}