package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
    FlywheelSim flywheelSim;
    SparkMaxSim flywheelSparkSim;
    SparkMax flywheelSpark;
    double setVoltage;

    public FlywheelIOSim() {
        flywheelSpark = new SparkMax(
                FlywheelConstants.FLYWHEEL_PORT,
                MotorType.kBrushless);
        flywheelSparkSim = new SparkMaxSim(flywheelSpark, DCMotor.getNEO(1));
        flywheelSim = new FlywheelSim (
            LinearSystemId.createFlywheelSystem (
                DCMotor.getNEO(1),
                FlywheelConstants.GEAR_RATIO,
                FlywheelConstants.MOMENT_OF_INERTIA
            ),
            DCMotor.getNEO(1)
        );
    }

    @Override
    public void setVoltage(double voltage) {
        this.flywheelSpark.setVoltage(voltage);
        this.flywheelSim.setInputVoltage(voltage);
        setVoltage = voltage;
    }

    @Override
    public double getVelocity() {
        return this.flywheelSim.getAngularVelocityRPM();
    }

    @Override
    public boolean hasCoral() {
        return true; // TODO: The coral state should be handled somewhere.
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocity = getVelocity();
        inputs.voltage = setVoltage;
    }

    @Override
    public void updateSimulation() {
        flywheelSim.update(0.02);
        // flywheelSparkSim.iterate();
    }
}