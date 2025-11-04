package frc.robot.io;

import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose; // This import is correct
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.VisionConstants;

public class VisionIOPhotonVision implements VisionIO {
    
    private final PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    public VisionIOPhotonVision() {
        System.out.println("[Init] Creating VisionIOPhotonVision");
        camera = new PhotonCamera(VisionConstants.kFrontCameraName);
        
        // Pose estimator will be created once the field layout is set
        poseEstimator = null;
    }

    @Override
    public void setFieldLayout(AprilTagFieldLayout layout) {
        if (layout != null) {
            poseEstimator = new PhotonPoseEstimator(
                layout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // This is a good default
                camera, 
                VisionConstants.kRobotToFrontCam
            );
        } else {
            poseEstimator = null;
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Get the latest result from the camera
        var result = camera.getLatestResult();
        inputs.hasTargets = result.hasTargets();
        
        // If pose estimator hasn't been initialized, return early
        if (poseEstimator == null) {
            inputs.observationPoses = new Pose3d[] {};
            inputs.observationTimestamps = new double[] {};
            inputs.observationAmbiguities = new double[] {};
            return;
        }

        // Get the estimated robot pose
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        
        if (estimatedPose.isPresent()) {
            // Valid pose found
            // --- FIX: Use .best, not .bestPose ---
            inputs.observationPoses = new Pose3d[] { estimatedPose.get().best }; 
            inputs.observationTimestamps = new double[] { estimatedPose.get().timestampSeconds };
            
            // --- FIX: Use .ambiguity, not .multiTagAmbig ---
            // This value is already the selected ambiguity (e.g., multi-tag or single-tag)
            inputs.observationAmbiguities = new double[] { estimatedPose.get().ambiguity };

        } else {
            // No pose calculated
            inputs.observationPoses = new Pose3d[] {};
            inputs.observationTimestamps = new double[] {};
            inputs.observationAmbiguities = new double[] {};
        }
    }
}

