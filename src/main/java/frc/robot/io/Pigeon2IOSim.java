package frc.robot.io;

import edu.wpi.first.wpilibj.Timer;

/**
 * Simulation implementation for the Pigeon 2.0 Gyro.
 * This class simulates the gyro's yaw based on commanded robot rotation.
 */
public class Pigeon2IOSim implements GyroIO {
    
    private double m_simYawDeg = 0.0;
    private double m_lastTimestamp = 0.0; // For internal dt calculation

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // --- THIS IS THE FIX ---
        // Removed the unused 'dt' variable.
        // The 'dt' for yaw calculation is handled in 'updateSim'.
        double now = Timer.getFPGATimestamp();
        m_lastTimestamp = now;
        // --- END FIX ---

        // In sim, the gyro is never calibrating
        inputs.isCalibrating = false;
        
        // Set all other inputs based on the simulated yaw
        inputs.yawDeg = m_simYawDeg;
        inputs.pitchDeg = 0.0; // Assume flat ground
        inputs.rollDeg = 0.0;  // Assume flat ground

        // Angular velocities are not critical for basic sim, but we log them
        // We can't get the "real" sim Z velocity here, so we'll just log 0.
        inputs.angularVelocityXRadPerSec = 0.0;
        inputs.angularVelocityYRadPerSec = 0.0;
        inputs.angularVelocityZRadPerSec = 0.0; 
        
        inputs.temperatureCelsius = 20.0; // Assume room temp
    }

    @Override
    public void setYaw(double yawDeg) {
        m_simYawDeg = yawDeg;
    }

    /**
     * Updates the gyro yaw based on the simulated rotation speed.
     * @param rotationRadsPerSec The simulated rotation speed of the robot.
     * @param dt The time delta (in seconds) since the last update.
     */
    public void updateSim(double rotationRadsPerSec, double dt) {
        // Integrate angular velocity to update yaw (in degrees)
        m_simYawDeg += Math.toDegrees(rotationRadsPerSec) * dt;

        // Keep yaw within -180 to 180
        if (m_simYawDeg > 180.0) {
            m_simYawDeg -= 360.0;
        } else if (m_simYawDeg < -180.0) {
            m_simYawDeg += 360.0;
        }
    }
}

