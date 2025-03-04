package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    public void setVoltage(double voltage);

    public default void simulationPeriodic() {
    };

    /**
     * @return Measured height of the elevator in meters.
     */
    public double getHeight();

    public default void periodic() {
    }

    public void zeroEncoders();
}
