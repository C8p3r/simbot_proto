package frc.robot.io;

import com.ctre.phoenix6.hardware.Pigeon2;
// Import StatusCode, not StatusCodeValue, from the root package
import com.ctre.phoenix6.StatusCode; 
import static frc.robot.Constants.DriveConstants.kPigeonCanId;

/**
 * Hardware implementation for the Pigeon 2.0 gyro using the CTRE Phoenix 6 API.
 */
public class Pigeon2IO implements GyroIO {
    private final Pigeon2 m_pigeon;

    public Pigeon2IO() {
        m_pigeon = new Pigeon2(kPigeonCanId);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // --- Populate all fields from the GyroIOInputs interface ---

        // Negate Yaw because WPILib treats positive angle as CCW, 
        // but Phoenix devices generally report CW positive.
        inputs.yawDeg = -m_pigeon.getYaw().getValueAsDouble();
        inputs.pitchDeg = m_pigeon.getPitch().getValueAsDouble();
        inputs.rollDeg = m_pigeon.getRoll().getValueAsDouble();

        // Convert angular velocities from degrees/sec to radians/sec
        // Negate Z velocity for WPILib convention
        inputs.angularVelocityXRadPerSec = Math.toRadians(m_pigeon.getAngularVelocityXWorld().getValueAsDouble());
        inputs.angularVelocityYRadPerSec = Math.toRadians(m_pigeon.getAngularVelocityYWorld().getValueAsDouble());
        inputs.angularVelocityZRadPerSec = -Math.toRadians(m_pigeon.getAngularVelocityZWorld().getValueAsDouble());

        inputs.temperatureCelsius = m_pigeon.getTemperature().getValueAsDouble();
        
        // FIX: Check the status of a primary signal (like Yaw) instead of getBootStatus().
        // If the signal is not "OK", we can assume it's still initializing.
        // Use StatusCode, not StatusCodeValue
        inputs.isCalibrating = m_pigeon.getYaw().getStatus() != StatusCode.OK;
    }

    @Override
    public void setYaw(double yawDeg) {
        m_pigeon.setYaw(-yawDeg); // Negate before setting
    }
}

