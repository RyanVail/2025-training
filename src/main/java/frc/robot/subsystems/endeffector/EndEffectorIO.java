package frc.robot.subsystems.endeffector;

public interface EndEffectorIO {
    public void setVoltage(double voltage);

    public double getAngle();

    public default void simulationPeriodic() {};
}
