package frc.robot.subsystems.endeffector;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOSim implements EndEffectorIO {
    private SingleJointedArmSim sim;

    public EndEffectorIOSim() {
        // A single jointed arm is a good subsitute to simulate our end effector.
        sim = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            EndEffectorConstants.GEAR_RATIO,
            EndEffectorConstants.MOMENT_OF_INERTIA,
            EndEffectorConstants.LENGTH,
            EndEffectorConstants.MIN_ANGLE,
            EndEffectorConstants.MAX_ANGLE,
            true,
            EndEffectorConstants.STARTING_ANGLE
        );
    }

    @Override
    public void simulationPeriodic() {
        sim.update(0.02);
    }

    @Override
    public void setVoltage(double voltage) {
        sim.setInputVoltage(voltage);
    }

    @Override
    public double getAngle() {
        return Units.radiansToDegrees(sim.getAngleRads());
    }
}
