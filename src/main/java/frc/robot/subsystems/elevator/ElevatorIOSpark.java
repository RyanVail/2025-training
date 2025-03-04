package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

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

    @Override
    public double getHeight() {
        Logger.recordOutput("Right spark Pos", rightSpark.getEncoder().getPosition());
        Logger.recordOutput("Left spark Pos", leftSpark.getEncoder().getPosition());
        return leftSpark.getEncoder().getPosition();
    }

    public void zeroEncoders() {
        leftSpark.getEncoder().setPosition(0.0);
        rightSpark.getEncoder().setPosition(0.0);
    }
}
