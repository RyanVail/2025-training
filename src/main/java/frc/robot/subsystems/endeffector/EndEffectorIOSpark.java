package frc.robot.subsystems.endeffector;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class EndEffectorIOSpark implements EndEffectorIO {
    private SparkMax spark;

    public EndEffectorIOSpark() {
        spark = new SparkMax(0, MotorType.kBrushless);
    }

    public void setVoltage(double voltage) {
        this.spark.setVoltage(voltage);
    }

    public double getAngle() {
        // TODO: Convert this to an angle.
        return spark.getAbsoluteEncoder().getPosition();
    }
}
