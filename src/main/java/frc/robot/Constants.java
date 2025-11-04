package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class is a repository for all configuration and tuning values.
 */
public final class Constants {

    // --- LOGGING/ADVANTAGEKIT CONSTANTS ---
    public static final boolean LOG_TO_RERUN = true; // Set to true to log to RERUN file
    public static final boolean REPLAY_LOGS = false; // Set to true to replay logs instead of running live

    // --- DRIVETRAIN CONSTANTS ---
    public static final class DriveConstants {
        // CAN IDs
        public static final int kFrontLeftDriveCanId = 1;
        public static final int kFrontLeftSteerCanId = 5;

        public static final int kFrontRightDriveCanId = 2;
        public static final int kFrontRightSteerCanId = 6;

        public static final int kRearLeftDriveCanId = 3;
        public static final int kRearLeftSteerCanId = 7;

        public static final int kRearRightDriveCanId = 4;
        public static final int kRearRightSteerCanId = 8;
        
        public static final int kPigeonCanId = 9; 

        // Chassis dimensions: distance from the center of the robot to the wheel center (in meters)
        public static final double kTrackWidth = Units.inchesToMeters(21.5);
        public static final double kWheelBase = Units.inchesToMeters(21.5);

        // Define the locations of the swerve modules relative to the robot's center (in meters)
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),     // Front Left
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),    // Front Right
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),    // Rear Left
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)    // Rear Right
        );

        // Max speeds (for simulation/testing)
        public static final double kMaxSpeedMetersPerSecond = 3.92; 
        public static final double kMaxAngularSpeedRadiansPerSecond = 6.0;
        
        // Feedforward gains for drive motor (Tuning required!)
        public static final double kDriveKs = 4; // Volts
        public static final double kDriveKv = 0.5; // Volts * seconds / meter

        // PID Constants for the Drive Motor Velocity Control (Tuning required!)
        public static final double kDriveP = 0.01;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;

        // PID Constants for the Steering Motor Position Control (Tuning required!)
        public static final double kSteerP = 10;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.0;
        
        // Gear ratios
        public static final double kSteerMotorGearRatio = 12.8; 
        public static final double kDriveMotorGearRatio = 8.14;

        // --- NEW SIMULATION CONSTANTS ---
        // Moment of Inertia (J) for simulation, in kg*m^2
        // This is a placeholder value; tune this for more accurate sim
        public static final double kDriveMotorInertia = 0.00001;
        public static final double kSteerMotorInertia = 0.00001;
        // --- END NEW CONSTANTS ---

        // Conversion factors to convert from raw sensor units (rotations) to meters or radians
        // The drive motor conversion needs to account for the wheel circumference
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kDriveConversionFactor = (kWheelDiameterMeters * Math.PI) / kDriveMotorGearRatio; 
        // The steer conversion is simply 2*Pi / Gear Ratio
        public static final double kSteerConversionFactor = (2 * Math.PI) / kSteerMotorGearRatio;
    }

    // --- JOYSTICK/CONTROLLER CONSTANTS ---
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.1;

        // --- NEW: Slew Rate Constants ---
        // (Units are "percent of full speed per second")
        // Tune these to get the "feel" you want
        public static final double kDriveSlewRate = 3; // 3.0 means it takes 0.33s to get from 0 to full speed
        public static final double kRotSlewRate = 1;   // 3.0 means it takes 0.33s to get from 0 to full rotation
    }
}

