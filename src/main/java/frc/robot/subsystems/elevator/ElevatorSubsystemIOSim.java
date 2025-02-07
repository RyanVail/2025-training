package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorSubsystemIOSim implements ElevatorSubsystemIO {
    ElevatorSim sim;

    public ElevatorSubsystemIOSim() {
        sim = new ElevatorSim(
                DCMotor.getNEO(2),
                Constants.ElevatorConstants.GEAR_RATIO,
                Constants.ElevatorConstants.MASS,
                Constants.ElevatorConstants.RADIUS,
                Constants.ElevatorConstants.MIN_HEIGHT,
                Constants.ElevatorConstants.MAX_HEIGHT,
                true,
                0.0);
    }

    @Override
    public void updateSimulation() {
        sim.update(0.02);
    }

    @Override
    public double getHeight() {
        return sim.getPositionMeters();
    }

    @Override
    public void setVoltage(double voltage) {
        sim.setInputVoltage(voltage);
    }
}
