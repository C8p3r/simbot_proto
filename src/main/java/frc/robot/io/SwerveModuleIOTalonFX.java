package frc.robot.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue; // NEW: Needed to explicitly set sensor source
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.DriveConstants.*;

/**
 * Hardware implementation for a swerve module using two Talon FX motor controllers.
 */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    private final TalonFX m_driveMotor;
    private final TalonFX m_steerMotor;

    // Control requests
    private final VelocityVoltage m_driveVelocityRequest = new VelocityVoltage(0);
    private final PositionDutyCycle m_steerPositionRequest = new PositionDutyCycle(0);

    /**
     * Constructs a SwerveModuleIOTalonFX.
     * @param driveMotorCanId The CAN ID of the drive motor.
     * @param steerMotorCanId The CAN ID of the steering motor.
     */
    public SwerveModuleIOTalonFX(int driveMotorCanId, int steerMotorCanId) {
        m_driveMotor = new TalonFX(driveMotorCanId);
        m_steerMotor = new TalonFX(steerMotorCanId);

        // --- Drive Motor (Kraken X60) Configuration ---
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Adjust as needed
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Configure the Drive Motor PID for velocity control (rotations per second)
        driveConfig.Slot0.kP = kDriveP;
        driveConfig.Slot0.kI = kDriveI;
        driveConfig.Slot0.kD = kDriveD;
        
        // FIX: Use SensorToMechanismRatio and explicitly set the sensor source
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.SensorToMechanismRatio = kDriveMotorGearRatio;
        
        m_driveMotor.getConfigurator().apply(driveConfig);

        // --- Steer Motor (Talon FX) Configuration ---
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Adjust as needed
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Configure the Steer Motor PID for position control (rotations)
        steerConfig.Slot0.kP = kSteerP;
        steerConfig.Slot0.kI = kSteerI;
        steerConfig.Slot0.kD = kSteerD;
        
        // FIX: Use SensorToMechanismRatio and explicitly set the sensor source
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        steerConfig.Feedback.SensorToMechanismRatio = kSteerMotorGearRatio;
        
        m_steerMotor.getConfigurator().apply(steerConfig);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Drive Motor Logging (Rotations -> Meters, Rotations/s -> Meters/s)
        inputs.drivePositionMeters = m_driveMotor.getPosition().getValueAsDouble() * kDriveConversionFactor;
        inputs.driveVelocityMetersPerSec = m_driveMotor.getVelocity().getValueAsDouble() * kDriveConversionFactor;
        inputs.driveAppliedVolts = m_driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveCurrentAmps = m_driveMotor.getSupplyCurrent().getValueAsDouble();

        // Steer Motor Logging (Rotations -> Radians, Rotations/s -> Radians/s)
        inputs.steerPositionRadians = m_steerMotor.getPosition().getValueAsDouble() * kSteerConversionFactor;
        inputs.steerVelocityRadiansPerSec = m_steerMotor.getVelocity().getValueAsDouble() * kSteerConversionFactor;
        inputs.steerAppliedVolts = m_steerMotor.getMotorVoltage().getValueAsDouble();
        inputs.steerCurrentAmps = m_steerMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setDriveVelocity(final double velocityMetersPerSecond, final double voltageVolts) {
        // Convert target velocity (m/s) to motor rotations per second
        final double targetDriveRps = velocityMetersPerSecond / kDriveConversionFactor;

        // Apply velocity control with feedforward
        m_driveMotor.setControl(m_driveVelocityRequest.withVelocity(targetDriveRps).withFeedForward(voltageVolts));
    }

    @Override
    public void setSteerAngle(double angleRadians) {
        // Convert the target angle (in radians) to motor rotations
        double targetSteerRotations = angleRadians / kSteerConversionFactor;
        m_steerMotor.setControl(m_steerPositionRequest.withPosition(targetSteerRotations));
    }

    @Override
    public void disable() {
        m_driveMotor.set(0);
        m_steerMotor.set(0);
    }
}
