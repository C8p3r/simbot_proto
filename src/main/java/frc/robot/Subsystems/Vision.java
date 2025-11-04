package frc.robot.subsystems;

import java.io.IOException;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.VisionIO;
import frc.robot.io.VisionIO.VisionIOInputs;

public class Vision extends SubsystemBase {
    
    private final VisionIO io;
    private final VisionIOInputs inputs = new VisionIOInputs();

    private AprilTagFieldLayout fieldLayout;
    private boolean isLayoutLoaded = false;

    /** Simple record class to store a single vision pose measurement. */
    public static record VisionMeasurement(Pose3d pose, double timestamp, double ambiguity) {}

    public Vision(VisionIO io) {
        System.out.println("[Init] Creating Vision Subsystem");
        this.io = io;
        
        // Load the AprilTag field layout
        try {
            // This is for the 2024 game. Update as needed for future games.
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            io.setFieldLayout(fieldLayout);
            isLayoutLoaded = true;
            System.out.println("[Init] AprilTag layout loaded successfully.");
        } catch (IOException e) {
            System.out.println("[Init] ERROR: Could not load AprilTag field layout! Vision will be disabled.");
            e.printStackTrace();
            isLayoutLoaded = false;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        // Log the estimated pose to AdvantageScope's 3D Field
        if (isLayoutLoaded && inputs.hasTargets && inputs.observationPoses.length > 0) {
            Logger.recordOutput("Vision/EstimatedPose", inputs.observationPoses[0].toPose2d());
        }
    }

    /**
     * Returns the latest valid vision-derived robot pose estimates.
     * @return An array of vision measurements, or an empty array if no valid measurements.
     */
    public VisionMeasurement[] getVisionMeasurements() {
        if (!isLayoutLoaded || !inputs.hasTargets || inputs.observationPoses.length == 0) {
            return new VisionMeasurement[0]; // No valid measurement
        }

        VisionMeasurement[] measurements = new VisionMeasurement[inputs.observationPoses.length];
        for (int i = 0; i < inputs.observationPoses.length; i++) {
            measurements[i] = new VisionMeasurement(
                inputs.observationPoses[i], 
                inputs.observationTimestamps[i],
                inputs.observationAmbiguities[i]
            );
        }
        return measurements;
    }

    /**
     * Returns the 2D pose of the robot based on the first (and likely only)
     * vision measurement. Returns null if no targets are visible.
     */
    public Pose2d getEstimated2DPose() {
        if (isLayoutLoaded && inputs.hasTargets && inputs.observationPoses.length > 0) {
            return inputs.observationPoses[0].toPose2d();
        }
        return null;
    }
}
