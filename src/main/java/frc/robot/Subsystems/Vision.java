package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;
import java.io.IOException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class Vision extends SubsystemBase {

    private final PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private AprilTagFieldLayout fieldLayout;

    // --- Simulation ---
    private VisionSystemSim visionSim;
    private final Field2d fieldSim = new Field2d();

    // --- Cached Results ---
    private Pose3d[] observationPoses = new Pose3d[] {};
    private double[] observationTimestamps = new double[] {};
    private double[] observationAmbiguities = new double[] {};
    private boolean hasTargets = false;

    /** Simple record class to store a single vision pose measurement. */
    public static record VisionMeasurement(Pose3d pose, double timestamp, double ambiguity) {}

    public Vision() {
        System.out.println("[Init] Creating Vision Subsystem");
        camera = new PhotonCamera(VisionConstants.kFrontCameraName);

        // Load the AprilTag field layout
        try {
            // This is for the 2024 game. Update as needed.
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
            
            // Create pose estimator
            poseEstimator = new PhotonPoseEstimator(
                fieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // A good default strategy
                VisionConstants.kRobotToFrontCam
            );
            
            System.out.println("[Init] AprilTag layout loaded successfully.");

        } catch (IOException e) {
            System.out.println("[Init] ERROR: Could not load AprilTag field layout! Vision will be disabled.");
            poseEstimator = null;
        }

        // --- Simulation Setup ---
        if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(fieldLayout);

            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(60)); // Example values
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
            
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, VisionConstants.kRobotToFrontCam);

            // Add rectangular outlines of the AprilTags to the field simulation
            for (var tag : fieldLayout.getTags()) {
                visionSim.getDebugField().getObject("Tag-" + tag.ID).setPose(tag.equals().toPose2d());
            }

            Logger.recordOutput("Vision/SimField", fieldSim);
        }
    }

    @Override
    public void periodic() {
        // This method is called every robot loop
        
        // Don't do anything if the pose estimator failed to load
        if (poseEstimator == null) {
            return;
        }

        // Get the latest result from the camera
        var result = camera.getLatestResult();
        this.hasTargets = result.hasTargets();

        // Get the estimated robot pose
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        
        if (estimatedPose.isPresent()) {
            // Valid pose found
            this.observationPoses = new Pose3d[] { estimatedPose.get().best }; 
            this.observationTimestamps = new double[] { estimatedPose.get().timestampSeconds };
            this.observationAmbiguities = new double[] { estimatedPose.get().ambiguity };
            
            // Log the estimated pose to AdvantageScope's 3D Field
            Logger.recordOutput("Vision/EstimatedPose", estimatedPose.get().best.toPose2d());
        } else {
            // No pose calculated
            this.observationPoses = new Pose3d[] {};
            this.observationTimestamps = new double[] {};
            this.observationAmbiguities = new double[] {};
        }

        // Log inputs
        Logger.recordOutput("Vision/HasTargets", this.hasTargets);
    }

    @Override
    public void simulationPeriodic() {
        // This method is only called in simulation
        // Update the vision simulation with the robot's "true" pose from odometry
        if (visionSim != null) {
            visionSim.update(m_odometry.getEstimatedPosition());
        }
    }

    /**
     * Updates the vision simulation with the robot's current pose.
     * @param robotPose The current pose of the robot.
     */
    public void simulationPeriodic(Pose2d robotPose) {
        if (visionSim != null) {
            visionSim.update(robotPose);
            
            // Update the sim field visualization
            fieldSim.setRobotPose(robotPose);
            
            // Draw vision estimates
            if (this.hasTargets && this.observationPoses.length > 0) {
                 fieldSim.getObject("VisionEstimation").setPose(this.observationPoses[0].toPose2d());
            } else {
                 fieldSim.getObject("VisionEstimation").setPoses();
            }
        }
    }

    /**
     * Returns the latest valid vision-derived robot pose estimates.
     * @return An array of vision measurements, or an empty array if no valid measurements.
     */
    public VisionMeasurement[] getVisionMeasurements() {
        if (!hasTargets || observationPoses.length == 0) {
            return new VisionMeasurement[0]; // No valid measurement
        }

        VisionMeasurement[] measurements = new VisionMeasurement[observationPoses.length];
        for (int i = 0; i < observationPoses.length; i++) {
            measurements[i] = new VisionMeasurement(
                observationPoses[i], 
                observationTimestamps[i],
                observationAmbiguities[i]
            );
        }
        return measurements;
    }

    /**
     * Returns the 2D pose of the robot based on the first (and likely only)
     * vision measurement. Returns null if no targets are visible.
     */
    public Pose2d getEstimated2DPose() {
        if (hasTargets && observationPoses.length > 0) {
            return observationPoses[0].toPose2d();
        }
        return null;
    }
}

