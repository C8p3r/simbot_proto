package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator; // <-- CORRECT IMPORT
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d; // Keep for gyro
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.io.GyroIO;
import frc.robot.io.GyroIO.GyroIOInputs;
import frc.robot.io.Pigeon2IOSim;

import static frc.robot.Constants.DriveConstants.*;

import frc.robot.subsystems.Vision.VisionMeasurement;

/**
 * The Drivetrain subsystem manages the swerve drive, odometry, and gyro.
 * It now also includes advanced, structured telemetry publishing and vision-based pose estimation.
 */
public class Drivetrain extends SubsystemBase {

    private final GyroIO m_gyroIO;
    private final GyroIOInputs m_gyroInputs = new GyroIOInputs();

    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_rearLeft;
    private final SwerveModule m_rearRight;

    // --- FIX: Use SwerveDrivePoseEstimator (2D) ---
    private final SwerveDrivePoseEstimator m_odometry;
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(); // Current chassis speeds
    private SwerveModuleState[] m_desiredModuleStates = new SwerveModuleState[] { // Desired module states
        new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    };
    
    private double m_lastSimTimestamp = 0.0;
    private double m_lastPeriodicTimestamp = 0.0; // For odometry frequency calculation

    // --- BEGIN TELEMETRY PUBLISHERS ---
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    // --- FIX: Publish a Pose2d ---
    private final StructPublisher<Pose2d> drivePosePub = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeedsPub = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStatesPub = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargetsPub = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositionsPub = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestampPub = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequencyPub = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };
    // --- END TELEMETRY PUBLISHERS ---

    private final Vision m_vision;

    /**
     * Creates a new Drivetrain subsystem.
     */
    public Drivetrain(GyroIO gyroIO, SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft, SwerveModule rearRight, Vision vision) {
        this.m_gyroIO = gyroIO;
        this.m_frontLeft = frontLeft;
        this.m_frontRight = frontRight;
        this.m_rearLeft = rearLeft;
        this.m_rearRight = rearRight;
        this.m_vision = vision;

        // --- FIX: Use SwerveDrivePoseEstimator (2D) ---
        m_odometry = new SwerveDrivePoseEstimator(
            kDriveKinematics,
            getRotation(), // Use 2D rotation
            getModulePositions(), // Use helper
            new Pose2d() // Start at (0, 0, 0)
        );

        // --- ADDED TELEMETRY INIT ---
        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
        // --- END ADDED TELEMETRY INIT ---
    }

    @Override
    public void periodic() {
        // Calculate periodic time
        double now = Timer.getFPGATimestamp();
        double dt = (m_lastPeriodicTimestamp == 0) ? 0.0 : now - m_lastPeriodicTimestamp;
        m_lastPeriodicTimestamp = now;

        // Update inputs from IO
        m_gyroIO.updateInputs(m_gyroInputs);
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();

        // Log inputs
        Logger.processInputs("Drive/Gyro", m_gyroInputs);

        // Update odometry
        SwerveModulePosition[] modulePositions = getModulePositions();
        // --- FIX: Use getRotation() (2D) for the update method ---
        m_odometry.update(getRotation(), modulePositions);
        
        // The Vision subsystem's periodic method will be called by the Scheduler,
        // which will update its internal state (including pose estimates).

        // Vision fusion
        VisionMeasurement[] measurements = m_vision.getVisionMeasurements();
        
        // Loop through all measurements
        for (VisionMeasurement measurement : measurements) {
            // Check ambiguity (lower is better)
            if (measurement != null && measurement.ambiguity() < 0.2) {
                // --- FIX: SwerveDrivePoseEstimator uses addVisionMeasurement with Pose2d ---
                m_odometry.addVisionMeasurement(
                    measurement.pose().toPose2d(), // Pass the Pose2d
                    measurement.timestamp()
                );
                // --- FIX: Log the 2D Pose from vision ---
                Logger.recordOutput("Drive/VisionPose", measurement.pose().toPose2d());
            }
        }

        // --- BEGIN STRUCTURED TELEMETRY PUBLISHING ---
        SwerveModuleState[] actualModuleStates = getModuleStates();
        
        // Publish all structured data
        drivePosePub.set(getPose()); // This will now publish the Pose2d
        driveSpeedsPub.set(m_chassisSpeeds);
        driveModuleStatesPub.set(actualModuleStates);
        driveModuleTargetsPub.set(m_desiredModuleStates);
        driveModulePositionsPub.set(modulePositions);
        driveTimestampPub.set(now);
        if (dt > 0.0) { // Avoid divide by zero
            driveOdometryFrequencyPub.set(1.0 / dt);
        }

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(actualModuleStates[i].angle);
            m_moduleDirections[i].setAngle(actualModuleStates[i].angle);
            // Scale length by max speed, with a 1.1 buffer so it doesn't max out
            m_moduleSpeeds[i].setLength(actualModuleStates[i].speedMetersPerSecond / (kMaxSpeedMetersPerSecond * 1.1));
        }
        // --- END STRUCTURED TELEMETRY PUBLISHING ---

        // Log the odometry pose to AdvantageKit
        Logger.recordOutput("Drive/Odometry", getPose());
    }

    /**
     * Drives the robot using ChassisSpeeds.
     * @param speeds The desired chassis speeds.
     * @param fieldRelative Whether the speeds are field-relative.
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        
        // --- SIMULATION SPEED MULTIPLIER ---
        if (RobotBase.isSimulation()) {
            speeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond , 
                speeds.vyMetersPerSecond , 
                speeds.omegaRadiansPerSecond
            );
        }
        // --- END SIMULATION SPEED MULTIPLIER ---

        this.m_chassisSpeeds = speeds;
        
        if (fieldRelative) {
            // Use the 2D rotation (yaw) for field-relative control
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation());
        }

        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeedMetersPerSecond);

        // Store desired states for telemetry
        this.m_desiredModuleStates = moduleStates;

        // --- ADDED LINE ---
        // Log the desired states array to AdvantageKit
        Logger.recordOutput("Drive/DesiredModuleStates", moduleStates);
        // --- END ADDED LINE ---

        m_frontLeft.setDesiredState(m_desiredModuleStates[0]);
        m_frontRight.setDesiredState(m_desiredModuleStates[1]);
        m_rearLeft.setDesiredState(m_desiredModuleStates[2]);
        m_rearRight.setDesiredState(m_desiredModuleStates[3]);
    }

    /**
     * Stops all swerve modules.
     */
    public void stop() {
        m_frontLeft.disable();
        m_frontRight.disable();
        m_rearLeft.disable();
        m_rearRight.disable();
        this.m_chassisSpeeds = new ChassisSpeeds();
        // Clear desired states
        // --- FIX: Corrected typo ---
        this.m_desiredModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
        };
    }

    /**
     * Returns the current 2D pose of the robot (position and rotation).
     */
    public Pose2d getPose() {
        // --- FIX: Use getEstimatedPosition() for PoseEstimator ---
        return m_odometry.getEstimatedPosition();
    }

    /**
     * Returns the current 2D rotation of the robot from the gyro (yaw).
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(m_gyroInputs.yawDeg);
    }

    /**
     * Returns the current 3D rotation of the robot from the gyro (roll, pitch, yaw).
     * This can be used for other purposes, even if odometry is 2D.
     */
    public Rotation3d getRotation3d() {
        return new Rotation3d(
            Units.degreesToRadians(m_gyroInputs.rollDeg),
            Units.degreesToRadians(m_gyroInputs.pitchDeg),
            Units.degreesToRadians(m_gyroInputs.yawDeg)
        );
    }

    /**
     * Helper method to get all module positions.
     * @return Array of SwerveModulePosition objects.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        };
    }

    /**
     * Returns the current *actual* states of the four swerve modules.
     * This is used for logging the actual state array.
     * @return An array of SwerveModuleState objects.
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            new SwerveModuleState(m_frontLeft.getVelocity(), m_frontLeft.getRotation()),
            new SwerveModuleState(m_frontRight.getVelocity(), m_frontRight.getRotation()), 
            new SwerveModuleState(m_rearLeft.getVelocity(), m_rearLeft.getRotation()),
            new SwerveModuleState(m_rearRight.getVelocity(), m_rearRight.getRotation())
        };
    }

    /**
     * Resets the odometry to a specific pose.
     * @param pose The 2D pose to reset to.
     */
    public void resetOdometry(Pose2d pose) { // <-- FIX: Changed to Pose2d
        m_odometry.resetPosition(
            getRotation(), // <-- FIX: Use getRotation()
            getModulePositions(), // Use helper
            pose
        );
    }

    /**
     * Zeros the gyro's heading and resets the odometry to the new heading.
     */
    public void zeroHeading() {
        m_gyroIO.setYaw(0.0);
        // Reset to the current translation but with a 0 rotation
        // --- FIX: Use Pose2d and Rotation2d ---
        resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d(0.0)));
    }

    @Override
    public void simulationPeriodic() {
        // --- ADDED SIMULATION dt CALCULATION ---
        double now = Timer.getFPGATimestamp();
        double dt = (m_lastSimTimestamp == 0) ? 0.02 : now - m_lastSimTimestamp;
        if (dt == 0.0) dt = 0.02; // Avoid 0 dt on first frame
        m_lastSimTimestamp = now;
        // --- END ADDED SECTION ---

        // Update the gyro simulation
        if (m_gyroIO instanceof Pigeon2IOSim) {
            // Pass dt to gyro sim
            ((Pigeon2IOSim) m_gyroIO).updateSim(m_chassisSpeeds.omegaRadiansPerSecond, dt);
        }
        
        // Update swerve module simulation
        // Pass dt to module sims
        m_frontLeft.simulationPeriodic(dt);
        m_frontRight.simulationPeriodic(dt);
        m_rearLeft.simulationPeriodic(dt);
        m_rearRight.simulationPeriodic(dt);

        // Update vision simulation
        m_vision.simulationPeriodic(getPose());
    }
}
