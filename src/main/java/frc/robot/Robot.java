package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.io.GyroIO;
import frc.robot.io.Pigeon2IO;
import frc.robot.io.Pigeon2IOSim;
import frc.robot.io.SwerveModuleIO;
import frc.robot.io.SwerveModuleIOSim;
import frc.robot.io.SwerveModuleIOTalonFX;

// Import the Vision subsystem, not the IO interface
import frc.robot.subsystems.Vision;

/**
 * The main Robot class, which extends AdvantageKit's LoggedRobot.
 * This class handles the initialization of AdvantageKit and the selection
 * of real vs. simulation IO implementations.
 */
public class Robot extends LoggedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    public Robot() {
        // Set the periodic update rate (default is 0.02s)
        super(0.02);
    }

/**
     * This function is run when the robot is first started up.
     */
    @Override
    public void robotInit() {
        // --- AdvantageKit Logger Initialization ---
        // (This part remains the same)
        Logger.recordMetadata("ProjectName", "SwerveDriveAK");
        

        // --- THIS IS THE CORRECT INITIALIZATION ---
        // Instantiate RobotContainer. This must be done AFTER IO selection.
        m_robotContainer = new RobotContainer(gyroIO, flModuleIO, frModuleIO, rlModuleIO, rrModuleIO, vision); // <-- PASS vision subsystem
    }

    @Override
    public void robotPeriodic() {
        // Runs the CommandScheduler (runs commands, calls subsystem periodic methods)
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
