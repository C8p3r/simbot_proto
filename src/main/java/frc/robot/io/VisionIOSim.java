package frc.robot.io;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionIOSim implements VisionIO {
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // In a simulation, vision data would be "faked" here.
        // For now, we'll just report that we see no targets.
        inputs.observationPoses = new Pose3d[] {};
        inputs.observationTimestamps = new double[] {};
        inputs.observationAmbiguities = new double[] {};
    }

    // No need to implement setFieldLayout here, as sim doesn't use it yet
}