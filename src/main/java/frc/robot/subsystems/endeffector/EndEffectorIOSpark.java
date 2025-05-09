package frc.robot.subsystems.endeffector;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.EndEffectorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EndEffectorIOSpark implements EndEffectorIO {
    private SparkMax spark;

    public EndEffectorIOSpark() {
        spark = new SparkMax(EndEffectorConstants.PORT, MotorType.kBrushless);
    }

    public void setVoltage(double voltage) {
        this.spark.setVoltage(voltage);
    }

    public double getAngle() {
        return spark.getEncoder().getPosition() * 10;
    }

    public void zeroEncoders() {
        spark.getEncoder().setPosition(0.0);
    }
}
