package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs {
        // A list of all measurements from all cameras this cycle
        public Pose3d[] observationPoses = new Pose3d[] {};
        public double[] observationTimestamps = new double[] {};
        public double[] observationAmbiguities = new double[] {};
    }

    /** Updates the inputs object with the latest data from the vision system. */
    public default void updateInputs(VisionIOInputs inputs) {}

    /** * Sets the field layout for the pose estimator.
     * This is necessary to know where the AprilTags are.
     */
    public default void setFieldLayout(AprilTagFieldLayout layout) {}
}