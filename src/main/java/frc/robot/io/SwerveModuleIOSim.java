package frc.robot.io;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// Import constants
import static frc.robot.Constants.DriveConstants.*;

/**
 * Simulation implementation for a Swerve Module.
 * This class simulates the drive and steer motors using PID controllers
 * and physics models (DCMotorSim).
 */
public class SwerveModuleIOSim implements SwerveModuleIO {

    private final DCMotorSim m_driveSim;
    private final DCMotorSim m_steerSim;

    private final PIDController m_drivePid;
    private final PIDController m_steerPid;
    
    // Conversion factors from rotations (sim output) to SI units
    private final double m_driveRotToMeters;
    private final double m_steerRotToRadians;

    public SwerveModuleIOSim() {
        
        DCMotor driveMotorModel = DCMotor.getKrakenX60(1);
        DCMotor steerMotorModel = DCMotor.getKrakenX60(1);

        // --- Drive Motor Simulation ---
        // This is the fix: Use the factory method from LinearSystemId
        m_driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveMotorModel,
                kDriveMotorInertia,
                kDriveMotorGearRatio
            ),
            driveMotorModel
        );

        // --- Steer Motor Simulation ---
        m_steerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                steerMotorModel,
                kSteerMotorInertia,
                kSteerMotorGearRatio
            ),
            steerMotorModel
        );

        // Initialize PID controllers
        m_drivePid = new PIDController(kDriveP, kDriveI, kDriveD);
        m_steerPid = new PIDController(kSteerP, kSteerI, kSteerD);
        m_steerPid.enableContinuousInput(-Math.PI, Math.PI); // Enable for angles

        // Calculate conversion factors
        m_driveRotToMeters = (kWheelDiameterMeters * Math.PI);
        m_steerRotToRadians = (2 * Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Use the correct getters from the DCMotorSim source code
        inputs.drivePositionMeters = m_driveSim.getAngularPositionRotations() * m_driveRotToMeters;
        inputs.driveVelocityMetersPerSec = (m_driveSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI)) * m_driveRotToMeters;
        inputs.driveAppliedVolts = m_driveSim.getInputVoltage();
        inputs.driveCurrentAmps = m_driveSim.getCurrentDrawAmps();

        // Steer Motor
        inputs.steerPositionRadians = m_steerSim.getAngularPositionRotations() * m_steerRotToRadians;
        inputs.steerVelocityRadiansPerSec = m_steerSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = m_steerSim.getInputVoltage();
        inputs.steerCurrentAmps = m_steerSim.getCurrentDrawAmps();
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSecond, double voltageVolts) {
        // The simulation update handles PID logic, so we just set the voltage
        m_driveSim.setInputVoltage(voltageVolts);
    }

    @Override
    public void setSteerAngle(double angleRadians) {
        // The simulation update handles PID logic, so we just set the voltage
        // (This will be set in updateSim)
    }

    @Override
    public void disable() {
        m_driveSim.setInputVoltage(0.0);
        m_steerSim.setInputVoltage(0.0);
    }
    
    /**
     * This is the main simulation update loop.
     * @param desiredState The desired state from the SwerveModule.
     * @param dt The time delta.
     */
    public void updateSim(SwerveModuleState desiredState, double dt) {
        
        // --- Drive Motor PID ---
        // Get current velocity from sim
        double currentVelocity = (m_driveSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI)) * m_driveRotToMeters;
        
        // Calculate PID output voltage
        double pidVolts = m_drivePid.calculate(
            currentVelocity, // Current velocity
            desiredState.speedMetersPerSecond // Desired velocity
        );
        
        // Calculate feedforward voltage
        double ffVolts = (new SimpleMotorFeedforward(kDriveKs, kDriveKv)).calculate(desiredState.speedMetersPerSecond);
        
        // Set the input voltage to the simulation
        m_driveSim.setInputVoltage(pidVolts + ffVolts);

        // --- Steer Motor PID ---
        // Get current angle from sim
        double currentAngle = m_steerSim.getAngularPositionRotations() * m_steerRotToRadians;
        
        // Calculate PID output voltage
        double steerVolts = m_steerPid.calculate(
            currentAngle, // Current angle
            desiredState.angle.getRadians() // Desired angle
        );
        
        // Set the input voltage to the simulation
        m_steerSim.setInputVoltage(steerVolts);

        // --- Update Physics ---
        m_driveSim.update(dt);
        m_steerSim.update(dt);
    }
}

