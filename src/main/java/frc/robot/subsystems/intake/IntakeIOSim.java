package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    FlywheelSim flywheelSim;
    SparkMaxSim flywheelSparkSim;
    SparkMax flywheelSpark;
    double setVoltage;

    public IntakeIOSim() {
        flywheelSpark = new SparkMax(
            IntakeConstants.PORT,
                MotorType.kBrushless);
        flywheelSparkSim = new SparkMaxSim(flywheelSpark, DCMotor.getNEO(1));
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getNEO(1),
                        IntakeConstants.GEAR_RATIO,
                        IntakeConstants.MOMENT_OF_INERTIA),
                DCMotor.getNEO(1));
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
    public double getVoltage() {
        return this.flywheelSim.getInputVoltage();
    }

    @Override
    public boolean hasCoral() {
        return true;
    }

    @Override
    public boolean isCoralLoaded() {
        return true;
    }

    public double getPosition() {
        return 0.0;
    }

    @Override
    public void simulationPeriodic() {
        flywheelSim.update(Constants.LOOP_TIME);
        // flywheelSparkSim.iterate();
    }
}