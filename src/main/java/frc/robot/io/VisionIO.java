package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs; // <-- IMPORT THIS

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    
    @AutoLog
    public static class VisionIOInputs implements LoggableInputs { // <-- ADDED IMPLEMENTS
        /** An array of all poses estimated this cycle. */
        public Pose3d[] observationPoses = new Pose3d[] {};
        
        /** An array of timestamps (FPGA) for each pose. */
        public double[] observationTimestamps = new double[] {};

        /** An array of ambiguity/error values for each pose. */
        public double[] observationAmbiguities = new double[] {};

        /** Indicates if the vision system is connected and seeing targets. */
        public boolean hasTargets = false;

        // --- ADD THESE METHODS TO SATISFY THE INTERFACE ---
        @Override
        public default void toLog(LogTable table) {
            // AdvantageKit's @AutoLog handles this automatically
        }

        @Override
        public default void fromLog(LogTable table) {
            // AdvantageKit's @AutoLog handles this automatically
        }
        // --- END OF ADDED METHODS ---
    }

    /** Updates the inputs object with new data from the vision system. */
    public default void updateInputs(VisionIOInputs inputs) {}

    /** * Sets the field layout for the pose estimator.
     * This is necessary to know where the AprilTags are.
     */
    public default void setFieldLayout(AprilTagFieldLayout layout) {}
}