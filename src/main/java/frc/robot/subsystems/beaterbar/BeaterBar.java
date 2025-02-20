package frc.robot.subsystems.beaterbar;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeaterBar extends SubsystemBase {
    BeaterBarIO io;
    double setSpeed;

    private static final String LPREFIX = "/Subsystems/BeaterBar/";

    public BeaterBar(BeaterBarIO io) {
        this.io = io;
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
        setSpeed = speed;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LPREFIX + "SetSpeed", setSpeed);
        Logger.recordOutput(LPREFIX + "Velocity", io.getVelocity());
        Logger.recordOutput(LPREFIX + "Voltage", io.getVoltage());
    }
}
