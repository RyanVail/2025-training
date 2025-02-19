package frc.robot.subsystems.intake;

public interface IntakeIO {
    public void setVoltage(double voltage);

    public double getVelocity();

    public double getVoltage();

    public boolean hasCoral();

    public boolean isCoralLoaded();

    public default void simulationPeriodic() {}
}
