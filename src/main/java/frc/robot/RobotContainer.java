package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Import IO interfaces
import frc.robot.io.GyroIO;
import frc.robot.io.SwerveModuleIO;
import frc.robot.io.VisionIO; // <-- ADD THIS

// Import Subsystems
import frc.robot.subsystems.Drivetrain; // <-- This will now be in subsystems package
import frc.robot.subsystems.SwerveModule; // <-- This will now be in subsystems package
import frc.robot.subsystems.Vision; // <-- ADD THIS

// Import commands
import frc.robot.commands.DriveCommand; // <-- This will now be in commands package

// Import constants
import static frc.robot.Constants.OIConstants.*;

/**
 * This class is the central "wiring" hub for the robot. It connects subsystems,
 * commands, and controller inputs.
 */
public class RobotContainer {
    
    // --- Subsystems ---
    private final Drivetrain m_drivetrain;
    private final Vision m_vision; // <-- ADD THIS

    // --- Controllers ---
    private final XboxController m_driverController = new XboxController(kDriverControllerPort);

    /**
     * The container for the robot.
     * @param gyroIO The Gyro IO implementation.
     * @param flIO The Front Left Swerve Module IO implementation.
     * @param frIO The Front Right Swerve Module IO implementation.
     * @param rlIO The Rear Left Swerve Module IO implementation.
     * @param rrIO The Rear Right Swerve Module IO implementation.
     * @param visionIO The Vision IO implementation. <-- ADD THIS
     */
    public RobotContainer(GyroIO gyroIO, SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO rlIO, SwerveModuleIO rrIO, VisionIO visionIO) { // <-- ADD visionIO
        
        // --- Instantiate Subsystems ---
        
        // Create SwerveModule subsystems (which take the IO implementation as a dependency)
        SwerveModule frontLeft = new SwerveModule(flIO, "FrontLeft");
        SwerveModule frontRight = new SwerveModule(frIO, "FrontRight");
        SwerveModule rearLeft = new SwerveModule(rlIO, "RearLeft");
        SwerveModule rearRight = new SwerveModule(rrIO, "RearRight");

        // Instantiate Vision subsystem
        m_vision = new Vision(visionIO); // <-- ADD THIS

        // Create the Drivetrain subsystem (which takes the Gyro and SwerveModules as dependencies)
        m_drivetrain = new Drivetrain(gyroIO, frontLeft, frontRight, rearLeft, rearRight, m_vision); // <-- PASS m_vision
        
        // --- Configure Default Commands ---
        
        // Set the Drivetrain's default command to the DriveCommand
        m_drivetrain.setDefaultCommand(
            new DriveCommand(
                m_drivetrain,
                // Supplier for X (forward/backward) speed
                () -> -applyDeadband(m_driverController.getLeftY(), kDriveDeadband),
                // Supplier for Y (left/right) speed
                () -> -applyDeadband(m_driverController.getLeftX(), kDriveDeadband),
                // Supplier for Rotation (turn) speed
                () -> -applyDeadband(m_driverController.getRightX(), kDriveDeadband),
                
                // --- THIS IS THE FIX ---
                // The 'true' value was replaced with a lambda (Supplier)
                // that reads the controller button.
                () -> !m_driverController.getLeftBumper() // Hold Left Bumper for Robot-Relative
            )
        );

        // --- Configure Button Bindings ---
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings.
     */
    private void configureButtonBindings() {
        // Example: Pressing the 'A' button zeroes the gyro
        new JoystickButton(m_driverController, XboxController.Button.kA.value)
            .onTrue(new InstantCommand(() -> m_drivetrain.zeroHeading(), m_drivetrain));
    }

    /**
     * Applies a deadband to a joystick axis value.
     * @param value The axis value (from -1.0 to 1.0).
     * @param deadband The deadband range (e.g., 0.1).
     * @return The value with the deadband applied, or 0.0 if within the deadband.
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return Math.signum(value) * (Math.abs(value) - deadband) / (1.0 - deadband);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous, or null if no command is selected.
     */
    public Command getAutonomousCommand() {
        // For now, return null (no autonomous command)
        // Example: return new AutoBalanceCommand(m_drivetrain);
        return null;
    }
}

