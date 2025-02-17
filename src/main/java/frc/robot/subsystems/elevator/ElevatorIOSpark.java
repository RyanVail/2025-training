package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSpark implements ElevatorIO {
    SparkMax spark;

    public ElevatorIOSpark() {
        spark = new SparkMax(ElevatorConstants.PORT, MotorType.kBrushless);
    }

    @Override
    public void setVoltage(double voltage) {
        spark.setVoltage(voltage);
    }

    @Override
    public double getHeight() {
        return spark.getEncoder().getPosition() / ElevatorConstants.METER_PER_ENCODER_UNIT;
    }
}
