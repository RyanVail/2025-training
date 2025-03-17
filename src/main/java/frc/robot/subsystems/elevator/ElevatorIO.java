package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    public void setVoltage(double voltage);

    public default void simulationPeriodic() {
    };

    public double getHeight();

    public default void periodic() {
    }

    public void zeroEncoders();
}
