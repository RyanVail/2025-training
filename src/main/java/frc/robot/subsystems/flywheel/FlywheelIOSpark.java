package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelIOSpark implements FlywheelIO {
    SparkMax spark;
    DigitalInput firstSensor;
    DigitalInput endSensor;

    public FlywheelIOSpark() {
        spark = new SparkMax(FlywheelConstants.PORT, MotorType.kBrushless);
        firstSensor = new DigitalInput(FlywheelConstants.FIRST_SENSOR_ID);
        endSensor = new DigitalInput(FlywheelConstants.END_SENSOR_ID);
    }

    @Override
    public void setVoltage(double voltage) {
        spark.setVoltage(voltage);
    }

    @Override
    public double getVelocity() {
        return spark.getEncoder().getVelocity();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocity = getVelocity();
        inputs.voltage = spark.getBusVoltage();
    }

    @Override
    public boolean hasCoral() {
        return firstSensor.get();
    }

    @Override
    public boolean isCoralLoaded() {
        return endSensor.get();
    }
}
