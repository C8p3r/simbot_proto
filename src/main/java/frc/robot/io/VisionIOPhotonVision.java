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

        // We can't create the pose estimator yet, because we need the AprilTag layout,
        // which is loaded asynchronously in the Vision subsystem.
        // We will create it when setFieldLayout() is called.
        poseEstimator = null;
    }

    @Override
    public void setFieldLayout(AprilTagFieldLayout layout) {
        if (layout != null) {
            poseEstimator = new PhotonPoseEstimator(
                layout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // Recommended strategy
                camera, 
                VisionConstants.kRobotToFrontCam
            );
        } else {
            poseEstimator = null; // Disable if layout is null
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if (poseEstimator == null) {
            inputs.observationPoses = new Pose3d[] {};
            inputs.observationTimestamps = new double[] {};
            inputs.observationAmbiguities = new double[] {};
            return;
        }

        var result = camera.getLatestResult();
        
        // Get all estimated robot poses from the camera
        ArrayList<Pose3d> poses = new ArrayList<>();
        ArrayList<Double> timestamps = new ArrayList<>();
        ArrayList<Double> ambiguities = new ArrayList<>();

        if (result.hasTargets()) {
            // Get the robot's pose relative to the field
            Optional<PhotonPoseEstimator.EstimatedPose> estimatedPose = poseEstimator.update(result);
            
            if (estimatedPose.isPresent()) {
                poses.add(estimatedPose.get().estimatedPose);
                timestamps.add(estimatedPose.get().timestampSeconds);
                
                // Get ambiguity of the *best* target used for this pose
                ambiguities.add(result.getBestTarget().getPoseAmbiguity());
            }
        }
        
        // Convert ArrayLists to arrays for logging
        inputs.observationPoses = poses.toArray(new Pose3d[poses.size()]);
        inputs.observationTimestamps = timestamps.stream().mapToDouble(d -> d).toArray();
        inputs.observationAmbiguities = ambiguities.stream().mapToDouble(d -> d).toArray();
    }
}