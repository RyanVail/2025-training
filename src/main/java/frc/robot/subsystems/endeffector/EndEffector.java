package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
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
    ArmFeedforward feedforward;
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

        this.feedforward = new ArmFeedforward(
                EndEffectorConstants.S,
                EndEffectorConstants.G,
                EndEffectorConstants.V);

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

        this.setSetpoint(EndEffectorConstants.IDLE_ANGLE);
    }

    @Override
    public void periodic() {
        // TODO: See if required.
        // if (setpoint <= Units.degreesToRadians(EndEffectorConstants.IDLE_ANGLE))
        //     setSetpoint(EndEffectorConstants.IDLE_ANGLE);

        lastStateSetpoint = EndEffectorConstants.PROFILE.calculate(0.02, lastStateSetpoint, stateSetpoint);

        double angle = Units.degreesToRadians(getAngle());
        double volts = this.pid.calculate(angle, lastStateSetpoint.position);
        volts += feedforward.calculate(angle, lastStateSetpoint.velocity);
        volts = Math.min(volts, RobotController.getBatteryVoltage());
        volts = Math.max(volts, -RobotController.getBatteryVoltage());
        io.setVoltage(volts);

        realLigament.setAngle(EndEffectorConstants.VISUALIZATION_BASE_ANGLE + getAngle());

        Logger.recordOutput(LPREFIX + "Setpoint", setpoint);
        Logger.recordOutput(LPREFIX + "Volts", volts);
        Logger.recordOutput(LPREFIX + "Angle", angle);

        // TODO: TMP!
        Logger.recordOutput(LPREFIX + "TVel", lastStateSetpoint.velocity);
        Logger.recordOutput(LPREFIX + "TPos", lastStateSetpoint.position);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void setSetpoint(double angle) {
        setpoint = Units.degreesToRadians(angle);
        stateSetpoint = new State(setpoint, 0.0);
        setpointLigament.setAngle(EndEffectorConstants.VISUALIZATION_BASE_ANGLE + angle);
    }

    public double getAngle() {
        return io.getAngle();
    }

    public void zeroEncoders() {
        io.zeroEncoders();
    }
}