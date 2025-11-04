package frc.robot.io;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionIOSim implements VisionIO {
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // In a simulation, we don't have a real camera.
        // We could fake data here, but for now, we'll just report no targets.
        inputs.observationPoses = new Pose3d[] {};
        inputs.observationTimestamps = new double[] {};
        inputs.observationAmbiguities = new double[] {};
        inputs.hasTargets = false;
    }

    // setFieldLayout() doesn't need to do anything in this simple sim
}