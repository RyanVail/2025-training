package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public class FlywheelIOInputs {
        double voltage = 0.0;
        double velocity = 0.0;
    }

    public void setVoltage(double voltage);

    /**
     * @return Measured flywheel velocity in units of rotations per minute
     */
    public double getVelocity();

    public boolean hasCoral();

    public boolean isCoralLoaded();

    public void updateInputs(FlywheelIOInputs inputs);

    public default void simulationPeriodic() {}
}
