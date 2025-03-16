package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

public class AlignPath extends AlignPose {
    Pose2d[] targets;
    int target_index;

    public AlignPath(Drive drive, Pose2d[] targets, AlignCamera camera) {
        super(
                drive,
                (targets == null || targets.length == 0) ? null : targets[0],
                camera);

        this.targets = targets;
    }

    public void setTargets(Pose2d[] targets) {
        this.targets = targets;
        if (targets != null && targets.length != 0)
            super.setTarget(targets[0]);
    }

    public void initialize() {
        this.target_index = 0;

        if (super.finished) {
            this.setTargets(this.targets);
            super.finished = false;
        }
    }

    @Override
    public boolean isFinished() {
        // Checking if the current point has been reached.
        if (!super.isFinished())
            return false;

        target_index++;

        // If the final point has been reached, the path is done.
        if (target_index >= targets.length)
            return true;

        // Moving to the next point because the path isn't done.
        super.setTarget(targets[target_index]);
        return false;
    }
}
