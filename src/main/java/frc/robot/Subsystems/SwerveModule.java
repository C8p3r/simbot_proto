package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.io.SwerveModuleIO;
import frc.robot.io.SwerveModuleIOInputs;
import frc.robot.io.SwerveModuleIOSim;

// Import constants using the full class path
import frc.robot.Constants.DriveConstants;

/**
 * The SwerveModule subsystem represents one of the four swerve modules.
 * It handles the logic for setting the desired state and logging its inputs.
 */
public class SwerveModule extends SubsystemBase {

    private final SwerveModuleIO m_io;
    private final SwerveModuleIOInputs m_inputs = new SwerveModuleIOInputs();
    private final String m_name;

    private final SimpleMotorFeedforward m_driveFeedforward = 
        new SimpleMotorFeedforward(DriveConstants.kDriveKs, DriveConstants.kDriveKv);
    
    private SwerveModuleState m_desiredState = new SwerveModuleState(); // Default to 0,0

    /**
     * Creates a new SwerveModule.
     * @param io The IO implementation (hardware or sim).
     * @param name The name of the module (e.g., "FrontLeft").
     */
    public SwerveModule(SwerveModuleIO io, String name) {
        this.m_io = io;
        this.m_name = name;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Drive/" + m_name, m_inputs);

        // Publish to SmartDashboard
        SmartDashboard.putNumber("Drive/" + m_name + "/SpeedMetersPerSec", m_inputs.driveVelocityMetersPerSec);
        SmartDashboard.putNumber("Drive/" + m_name + "/AngleRad", m_inputs.steerPositionRadians);
    }

    /**
     * Sets the desired state for the module (speed and angle).
     * @param desiredState The desired state.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the state to minimize turning
        // The static .optimize() method returns the new, optimized state.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getRotation());
        
        // Assign the now-optimized state to our class member
        m_desiredState = optimizedState;

        // Calculate feedforward voltage
        double ffVolts = m_driveFeedforward.calculate(m_desiredState.speedMetersPerSecond);

        // Send commands to IO layer
        m_io.setDriveVelocity(m_desiredState.speedMetersPerSecond, ffVolts);
        m_io.setSteerAngle(m_desiredState.angle.getRadians());
    }

    /** Disables the module (sets voltage to 0). */
    public void disable() {
        m_io.disable();
        m_desiredState = new SwerveModuleState();
    }

    /** Returns the current position of the module. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_inputs.driveVelocityMetersPerSec, Rotation2d.fromRadians(m_inputs.steerPositionRadians));
    }

    /** Returns the current rotation of the module. */
    public Rotation2d getRotation() {
        return Rotation2d.fromRadians(m_inputs.steerPositionRadians);
    }

    // --- NEW METHOD ---
    /** Returns the current velocity of the module. */
    public double getVelocity() {
        return m_inputs.driveVelocityMetersPerSec;
    }
    // --- END NEW METHOD ---

    /**
     * This is the crucial simulation link.
     * @param dt The simulation delta time.
     */
    public void simulationPeriodic(double dt) {
        if (m_io instanceof SwerveModuleIOSim) {
            // Pass the desired state and dt to the simulation
            ((SwerveModuleIOSim) m_io).updateSim(m_desiredState, dt);
        }
    }
}

