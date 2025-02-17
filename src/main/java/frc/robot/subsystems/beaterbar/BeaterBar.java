package frc.robot.subsystems.beaterbar;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeaterBar extends SubsystemBase {
    public BeaterBarIO io;

    public BeaterBar(BeaterBarIO io) {
        this.io = io;
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }
}
