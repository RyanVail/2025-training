package frc.robot.subsystems.endeffector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOSpark implements EndEffectorIO {
    private SparkMax spark;

    public EndEffectorIOSpark() {
        spark = new SparkMax(0, MotorType.kBrushless);
    }

    public void periodic() {
    }

    public void setVoltage(double voltage) {
        this.spark.setVoltage(voltage);
    }

    public double getAngle() {
        return Units.degreesToRadians(2 * Math.PI * spark.getAbsoluteEncoder().getPosition()
                / (EndEffectorConstants.GEAR_RATIO * EndEffectorConstants.ENCODER_RESOLUTION));
    }
}
