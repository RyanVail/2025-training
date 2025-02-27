package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.EndEffectorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EndEffectorIOSpark implements EndEffectorIO {
    private SparkMax spark;

    public EndEffectorIOSpark() {
        spark = new SparkMax(EndEffectorConstants.PORT, MotorType.kBrushless);

        // TODO: Make a better way to do this.
        spark.getEncoder().setPosition(0);
    }

    public void periodic() {
        Logger.recordOutput("EndEffectorAngle", spark.getEncoder().getPosition());
    }

    public void setVoltage(double voltage) {
        this.spark.setVoltage(voltage);
    }

    public double getAngle() {
        return spark.getEncoder().getPosition();
    }

    public void zeroEncoders() {
        spark.getEncoder().setPosition(0.0);
    }
}
