package frc.robot.subsystems;

import java.io.IOException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.VisionIO;
import frc.robot.io.VisionIO.VisionIOInputs;

public class Vision extends SubsystemBase {
    
    private final VisionIO io;
    private final VisionIOInputs inputs = new VisionIOInputs();

    private AprilTagFieldLayout fieldLayout;
    private boolean isLayoutLoaded = false;

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
    }

    /**
     * Returns the latest vision-derived robot pose estimate.
     * @return The estimated pose (Pose3d) and the timestamp (in seconds).
     */
    public VisionMeasurement getVisionMeasurement() {
        if (!isLayoutLoaded || inputs.observationPoses.length == 0) {
            return null; // No valid measurement
        }

        // For now, just return the first observation
        // We could add logic here to filter for low ambiguity
        return new VisionMeasurement(
            inputs.observationPoses[0], 
            inputs.observationTimestamps[0],
            inputs.observationAmbiguities[0]
        );
    }

    /** Simple record class to store a single vision pose measurement. */
    public static record VisionMeasurement(Pose3d pose, double timestamp, double ambiguity) {}
}