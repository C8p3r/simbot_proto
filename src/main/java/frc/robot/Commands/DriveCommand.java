package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;

/**
 * A command for teleoperated swerve drive.
 */
public class DriveCommand extends Command {
    private final Drivetrain m_drivetrain;
    private final Supplier<Double> m_xSpeed;
    private final Supplier<Double> m_ySpeed;
    private final Supplier<Double> m_rotSpeed;
    private final Supplier<Boolean> m_isFieldRelative;

    // Slew rate limiters to smooth joystick inputs
    // --- THIS IS THE FIX ---
    // Use kDriveSlewRate for linear motion and kRotSlewRate for rotation.
    private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(kDriveSlewRate);
    private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(kDriveSlewRate);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(kRotSlewRate);
    // --- END FIX ---


    /**
     * Creates a new DriveCommand.
     *
     * @param drivetrain The drivetrain subsystem.
     * @param xSpeed Supplier for the x speed (strafe).
     * @param ySpeed Supplier for the y speed (forward/backward).
     * @param rotSpeed Supplier for the rotational speed.
     * @param isFieldRelative Supplier for whether driving is field-relative.
     */
    public DriveCommand(Drivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> ySpeed, 
                        Supplier<Double> rotSpeed, Supplier<Boolean> isFieldRelative) {
        m_drivetrain = drivetrain;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rotSpeed = rotSpeed;
        m_isFieldRelative = isFieldRelative;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Get joystick values
        double xSpeed = m_xSpeed.get();
        double ySpeed = m_ySpeed.get();
        double rotSpeed = m_rotSpeed.get();

        // Apply deadband
        xSpeed = Math.abs(xSpeed) > kDriveDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > kDriveDeadband ? ySpeed : 0.0;
        rotSpeed = Math.abs(rotSpeed) > kDriveDeadband ? rotSpeed : 0.0;

        // Apply slew rate limiting
        xSpeed = m_xLimiter.calculate(xSpeed) * kMaxSpeedMetersPerSecond;
        ySpeed = m_yLimiter.calculate(ySpeed) * kMaxSpeedMetersPerSecond;
        rotSpeed = m_rotLimiter.calculate(rotSpeed) * kMaxAngularSpeedRadiansPerSecond;

        // Create ChassisSpeeds object
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

        // Pass the boolean supplier's value to the drive method
        m_drivetrain.drive(chassisSpeeds, m_isFieldRelative.get());
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }
}
