package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
    FlywheelIO io;
    FlywheelIOInputsAutoLogged inputs;

    PIDController pid;
    SimpleMotorFeedforward feedforward;

    private static final String LPREFIX = "/Subsystems/Flywheel/";

    public Flywheel(FlywheelIO io) {
        this.io = io;
        inputs = new FlywheelIOInputsAutoLogged();
        pid = new PIDController(
                Constants.FlywheelConstants.P,
                Constants.FlywheelConstants.I,
                Constants.FlywheelConstants.D);
        feedforward = new SimpleMotorFeedforward(
                Constants.FlywheelConstants.S,
                Constants.FlywheelConstants.V,
                Constants.FlywheelConstants.A);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(LPREFIX + "Inputs", inputs);
        Logger.recordOutput(LPREFIX + "HasCoral", hasCoral());
        Logger.recordOutput(LPREFIX + "IsCoralLoaded", isCoralLoaded());

        io.simulationPeriodic();

        double vel = inputs.velocity;
        double volts = pid.calculate(vel) + feedforward.calculate(vel);
        io.setVoltage(volts);
    }

    public void setVelocitySetpoint(double velocity) {
        pid.setSetpoint(velocity);
    }

    public boolean hasCoral() {
        return io.hasCoral();
    }

    public boolean isCoralLoaded() {
        return io.isCoralLoaded();
    }
}
