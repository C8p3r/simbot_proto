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

import frc.robot.io.VisionIO;
import frc.robot.io.VisionIOPhotonVision;
import frc.robot.io.VisionIOSim;

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
        
        if (isReal() && Constants.LOG_TO_RERUN) {
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
            Logger.addDataReceiver(new NT4Publisher());
        } else if (isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
        }
        if (Constants.REPLAY_LOGS) {
            String path = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(path));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        }
        Logger.start(); // Start logging!

        // --- IO Implementation Selection ---
        final GyroIO gyroIO;
        final SwerveModuleIO flModuleIO;
        final SwerveModuleIO frModuleIO;
        final SwerveModuleIO rlModuleIO;
        final SwerveModuleIO rrModuleIO;
        final VisionIO visionIO; // <-- ADD THIS

        if (RobotBase.isReal()) {
            // Real robot IO
            gyroIO = new Pigeon2IO();
            flModuleIO = new SwerveModuleIOTalonFX(Constants.DriveConstants.kFrontLeftDriveCanId, Constants.DriveConstants.kFrontLeftSteerCanId);
            frModuleIO = new SwerveModuleIOTalonFX(Constants.DriveConstants.kFrontRightDriveCanId, Constants.DriveConstants.kFrontRightSteerCanId);
            rlModuleIO = new SwerveModuleIOTalonFX(Constants.DriveConstants.kRearLeftDriveCanId, Constants.DriveConstants.kRearLeftSteerCanId);
            rrModuleIO = new SwerveModuleIOTalonFX(Constants.DriveConstants.kRearRightDriveCanId, Constants.DriveConstants.kRearRightSteerCanId);

            visionIO = new VisionIOPhotonVision(); // <-- ADD THIS

        } else {
            // Simulation IO
            gyroIO = new Pigeon2IOSim();
            flModuleIO = new SwerveModuleIOSim();
            frModuleIO = new SwerveModuleIOSim();
            rlModuleIO = new SwerveModuleIOSim();
            rrModuleIO = new SwerveModuleIOSim();

            visionIO = new VisionIOSim(); // <-- ADD THIS
        }

        // --- THIS IS THE CORRECT INITIALIZATION ---
        // Instantiate RobotContainer. This must be done AFTER IO selection.
        m_robotContainer = new RobotContainer(gyroIO, flModuleIO, frModuleIO, rlModuleIO, rrModuleIO, visionIO); // <-- PASS visionIO
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

