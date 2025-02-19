package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSpark implements ElevatorIO {
    SparkMax leftSpark;
    SparkMax rightSpark;

    public ElevatorIOSpark() {
        leftSpark = new SparkMax(ElevatorConstants.LEFT_PORT, MotorType.kBrushless);
        rightSpark = new SparkMax(ElevatorConstants.RIGHT_PORT, MotorType.kBrushless);
    }

    @Override
    public void setVoltage(double voltage) {
        leftSpark.setVoltage(voltage);
        rightSpark.setVoltage(-voltage);
    }

    // TODO: The inital value has to be calibrated.
    @Override
    public double getHeight() {
        return leftSpark.getEncoder().getPosition();// * ElevatorConstants.METER_PER_ENCODER_UNIT;
    }
}
