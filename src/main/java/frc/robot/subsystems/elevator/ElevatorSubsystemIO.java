package frc.robot.subsystems.elevator;

public interface ElevatorSubsystemIO {
    public void setVoltage(double voltage);

    public default void updateSimulation() {};

    /**
     * @return Measured height of the elevator in meters.
     */
    public double getHeight();
}
