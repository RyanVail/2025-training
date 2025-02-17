package frc.robot.subsystems.beaterbar;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.BeaterBarConstants;

public class BearterBarIOFlex implements BeaterBarIO {
    SparkFlex flex;

    public BearterBarIOFlex() {
        flex = new SparkFlex(BeaterBarConstants.PORT, MotorType.kBrushless);
    }

    @Override
    public void setSpeed(double speed) {
        flex.set(speed);
    }
}
