package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSpark implements IntakeIO {
    SparkMax spark;
    DigitalInput firstSensor;
    DigitalInput endSensor;

    public IntakeIOSpark() {
        spark = new SparkMax(IntakeConstants.PORT, MotorType.kBrushless);
        firstSensor = new DigitalInput(IntakeConstants.FIRST_SENSOR_ID);
        endSensor = new DigitalInput(IntakeConstants.END_SENSOR_ID);
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
    public double getVoltage() {
        return spark.getBusVoltage();
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
