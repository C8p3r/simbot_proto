package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.*;; // ADDED: Required import

/**
 * An interface for the Gyro, enabling AdvantageKit logging and simulation.
 * Any class that talks to the hardware (Pigeon 2.0) or provides simulation data must implement this.
 */
public interface GyroIO {
    
    /** Gyro data that will be logged and can be used in the Drivetrain. */
    @AutoLog // AdvantageKit will automatically log all public fields
    public class GyroIOInputs implements LoggableInputs { // FIX: Must implement LoggableInputs
        public double yawDeg = 0.0;
        public double pitchDeg = 0.0;
        public double rollDeg = 0.0;
        public double angularVelocityXRadPerSec = 0.0;
        public double angularVelocityYRadPerSec = 0.0;
        public double angularVelocityZRadPerSec = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean isCalibrating = false;
        
        /** This method is required by the LoggableInputs interface. */
        public void toLog(org.littletonrobotics.junction.LogTable table) {
            // AdvantageKit's @AutoLog handles this automatically
        }

        /** This method is required by the LoggableInputs interface. */
        public void fromLog(org.littletonrobotics.junction.LogTable table) {
            // AdvantageKit's @AutoLog handles this automatically
        }
    }

    /** Updates the inputs object with new data from the hardware. */
    public default void updateInputs(GyroIOInputs inputs) {}

    /** Sets the current yaw angle of the gyro (used for zeroing). */
    public default void setYaw(double yawDeg) {}
}

