package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;

public class AutoCommands {
    public static Command scoreAlgae(ElevatorSubsystem elevator, Flywheel flywheel) {
        return flywheel.setVelocity(FlywheelConstants.ALGAE_SCORE_VEL)
                .andThen(new WaitCommand(FlywheelConstants.ALGAE_SCORE_TIME))
                .andThen(flywheel.setVelocity(0.0));
    }
}