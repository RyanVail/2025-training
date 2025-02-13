package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public class FlywheelIOInputs {
        double voltage = 0.0;
        double velocity = 0.0;
    }

    /**
     * Sets the desired RPM of the flywheel
     * 
     * @param setpoint Prescribed flywheel velocity,
     *                 in units of rotations per minute
     */
    public void setVoltage(double setpoint);

    /**
     * @return Measured flywheel velocity in units of rotations per minute
     */
    public double getVelocity();

    public boolean hasCoral();

    public void updateInputs(FlywheelIOInputs inputs);

    public default void simulationPeriodic() {}
}
