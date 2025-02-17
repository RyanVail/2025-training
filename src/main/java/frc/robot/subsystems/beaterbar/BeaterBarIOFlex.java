package frc.robot.subsystems.beaterbar;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.BeaterBarConstants;

public class BeaterBarIOFlex implements BeaterBarIO {
    SparkFlex flex;

    public BeaterBarIOFlex() {
        flex = new SparkFlex(BeaterBarConstants.PORT, MotorType.kBrushless);
    }

    @Override
    public void setSpeed(double speed) {
        flex.set(speed);
    }

    @Override
    public double getVelocity() {
        return flex.getEncoder().getVelocity();
    }
}
