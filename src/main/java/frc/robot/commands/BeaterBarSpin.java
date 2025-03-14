package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beaterbar.BeaterBar;

public class BeaterBarSpin extends Command {
    BeaterBar bar;
    double speed;

    public BeaterBarSpin(BeaterBar bar, double speed) {
        this.bar = bar;
        this.speed = speed;
    }

    public void initialize() {
        this.bar.setSpeed(this.speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.bar.setSpeed(0.0);
    }
}
