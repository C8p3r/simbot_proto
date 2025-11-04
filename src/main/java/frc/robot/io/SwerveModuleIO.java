package frc.robot.io;

/**
 * An interface for a Swerve Module, enabling AdvantageKit logging and simulation.
 */
public interface SwerveModuleIO {

    /** Updates the inputs object with new data from the hardware. */
    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    /**
     * Sets the desired drive velocity.
     * @param velocityMetersPerSecond Desired velocity.
     * @param voltageVolts Feedforward voltage.
     */
    public default void setDriveVelocity(double velocityMetersPerSecond, double voltageVolts) {}

    /**
     * Sets the desired steer angle.
     * @param angleRadians Desired angle.
     */
    public default void setSteerAngle(double angleRadians) {}

    /** Disables the module, setting voltage to 0. */
    public default void disable() {}
}

