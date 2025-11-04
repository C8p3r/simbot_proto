package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {

    // A bot-mounted camera is defined by its name and its position/rotation relative to
    // the robot's center (the center of rotation of the swerve drive).

    // --- CAMERA 1 ("FrontCam") ---
    public static final String kFrontCameraName = "photonvision"; // <-- Change this to your camera's name in the PhotonVision UI

    // The transform from the robot's center to the camera's optical center.
    // This is a critical value to measure precisely.
    // - X: Positive is forward
    // - Y: Positive is left
    // - Z: Positive is up
    // REMEMBER: These are in METERS! Use Units.inchesToMeters() for clarity.
    public static final Transform3d kRobotToFrontCam = 
        new Transform3d(
            new Translation3d(Units.inchesToMeters(12.0), Units.inchesToMeters(0.0), Units.inchesToMeters(10.0)), 
            new Rotation3d(0.0, 0.0, 0.0) // No rotation relative to robot frame
        );

    // --- (Optional) CAMERA 2 ("RearCam") ---
    // public static final String kRearCameraName = "photonvision_rear";
    // public static final Transform3d kRobotToRearCam = 
    //     new Transform3d(
    //         new Translation3d(Units.inchesToMeters(-12.0), Units.inchesToMeters(0.0), Units.inchesToMeters(10.0)),
    //         new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)) // Rotated 180 deg
    //     );
    
    // --- POSE ESTIMATION STANDARD DEVIATIONS ---
    // These are ballpark values to start with. Tune these based on actual robot performance.
    // Standard deviations (x, y, and theta) for single-tag poses. 
    // These should be higher, as single-tag poses are less reliable.
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(30));

    // Standard deviations (x, y, and theta) for multi-tag poses.
    // These should be lower, as multi-tag poses are more reliable.
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.4, 0.4, Units.degreesToRadians(15));
}
