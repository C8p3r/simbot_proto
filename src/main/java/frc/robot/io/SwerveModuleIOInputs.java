package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.*;
import org.littletonrobotics.junction.LogTable;

/**
 * SwerveModule data that will be logged, now outside the interface for easier logging access.
 */
@AutoLog
public class SwerveModuleIOInputs implements LoggableInputs {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public double steerPositionRadians = 0.0;
    public double steerVelocityRadiansPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;

    /** This method is required by the LoggableInputs interface. */
    public void toLog(LogTable table) {
        // AdvantageKit's @AutoLog handles this automatically
    }

    /** This method is required by the LoggableInputs interface. */
    public void fromLog(LogTable table) {
        // AdvantageKit's @AutoLog handles this automatically
    }
}

