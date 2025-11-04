package frc.robot.io;

import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
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
        
        // If we have no targets, or if the pose estimator hasn't been initialized, return early
        if (!inputs.hasTargets || poseEstimator == null) {
            inputs.observationPoses = new Pose3d[] {};
            inputs.observationTimestamps = new double[] {};
            inputs.observationAmbiguities = new double[] {};
            return;
        }

        // Get the estimated robot pose
        Optional<PhotonPoseEstimator.EstimatedPose> estimatedPose = poseEstimator.update();
        
        if (estimatedPose.isPresent()) {
            // Valid pose found
            inputs.observationPoses = new Pose3d[] { estimatedPose.get().estimatedPose };
            inputs.observationTimestamps = new double[] { estimatedPose.get().timestampSeconds };
            
            // Get ambiguity of the *best* target used for this pose
            double bestAmbiguity = 9999.0;
            if (result.getBestTarget() != null) {
                bestAmbiguity = result.getBestTarget().getPoseAmbiguity();
            }
            inputs.observationAmbiguities = new double[] { bestAmbiguity };

        } else {
            // No pose calculated
            inputs.observationPoses = new Pose3d[] {};
            inputs.observationTimestamps = new double[] {};
            inputs.observationAmbiguities = new double[] {};
        }
    }
}