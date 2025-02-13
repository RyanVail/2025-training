package frc.robot.subsystems.endeffector;

public interface EndEffectorIO {
    public void setVoltage(double voltage);

    public double getAngle();

    public void periodic();

    public default void simulationPeriodic() {};
}
