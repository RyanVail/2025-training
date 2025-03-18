package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOSim implements EndEffectorIO {
    SingleJointedArmSim sim;

    public EndEffectorIOSim() {
        sim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                EndEffectorConstants.GEAR_RATIO,
                SingleJointedArmSim.estimateMOI(EndEffectorConstants.LENGTH, EndEffectorConstants.MASS),
                EndEffectorConstants.LENGTH,
                EndEffectorConstants.MIN_ANGLE,
                EndEffectorConstants.MAX_ANGLE + EndEffectorConstants.STARTING_ANGLE,
                true,
                EndEffectorConstants.STARTING_ANGLE);
    }

    @Override
    public void simulationPeriodic() {
        sim.update(Constants.LOOP_TIME);
    }

    @Override
    public void setVoltage(double voltage) {
        sim.setInputVoltage(voltage);
    }

    @Override
    public double getAngle() {
        return Units.radiansToDegrees(sim.getAngleRads() - EndEffectorConstants.STARTING_ANGLE);
    }

    public void zeroEncoders() {
    }
}
