package frc.robot;

import com.studica.frc.MockDS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.autonomous.AutoFlow;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private RobotContainer m_robotContainer;
    private Command autonomousCommand;

    private boolean active = false;
    private int countLED;
    private boolean prevLEDValue;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        RobotContainer.controlpanel.setRunningLED(false);
        RobotContainer.controlpanel.setStoppedLED(false);
        countLED = 1;
        prevLEDValue = true;
    
        if (Constants.ENABLEMOCKDS) {
            RobotContainer.controlpanel.mockDS();
        }

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        if (Constants.ENABLEMOCKDS) {
            // If reset buttion is pressed, reset odom
            // if (RobotContainer.controlpanel.getResetButton()) {
            //     RobotContainer.drivetrain.initialise();;
            // }
            // If Start button pushed enabled robot in auto mode
            if ((RobotContainer.controlpanel.getStartButton() && !active) || (RobotContainer.controlpanel.getStartButton() && !RobotContainer.controlpanel.isEnabled()))
            {
                RobotContainer.controlpanel.enableMockDS();
                active = true;
            }
            // If E-Stop button is pushed disable the robot
            if (RobotContainer.controlpanel.getStopButton() && active)
            {
                RobotContainer.controlpanel.disableMockDS();
                active = false;
            }

        }
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        /** This if statement checks if an autochooser has been created */
        if (null == RobotContainer.autoChooser)
        {
            RobotContainer.autoChooser = new SendableChooser<>();
        }

        /** Creates and Assigns "AutoFlow" as the Default Autonomous Option */
        RobotContainer.autoChooser.setDefaultOption("AutoFlow", "AutoFlow");
        RobotContainer.autoMode.put("AutoFlow", new AutoFlow());

        /** Add other Modes to the chooser list */
        addAutoMode(RobotContainer.autoChooser, "AutoFlow", new AutoFlow()); // rename <AutoFlow> to the respective autonomous command file (eg: "CoreTask", new CoreTask() )

        /** Adds chooser interface to Smartdashboard */
        SmartDashboard.putData(RobotContainer.autoChooser);
        
        //? This is an example of automode creation
        /**
        RobotContainer.autoChooser.setDefaultOption("Drive Forward", "Drive Forward");
        RobotContainer.autoMode.put("Drive Forward", new DriveForward());
        addAutoMode(RobotContainer.autoChooser, "Drive Forward with PID", new DriveForwardWithPID());
        SmartDashboard.putData(RobotContainer.autoChooser);
        */

        RobotContainer.controlpanel.setRunningLED(false);
        RobotContainer.controlpanel.setStoppedLED(true);
    }

    // Used to create another autoMode and add it to the chooser
    public void addAutoMode(SendableChooser<String> chooser, String auto, AutoCommand cmd)
    {
        chooser.addOption(auto, auto);
        RobotContainer.autoMode.put(auto, cmd);
    }

    @Override
    public void disabledPeriodic() {
        String mode = RobotContainer.autoChooser.getSelected();
        SmartDashboard.putString("Chosen Auto Mode", mode);
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {

        RobotContainer.controlpanel.setStoppedLED(false);

        autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
        autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        runningLED();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
        autonomousCommand.cancel();
        }
        RobotContainer.controlpanel.setStoppedLED(false);

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        runningLED();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * added here to satisfy the watchdog
     */
    @Override
    public void simulationInit(){
    }
  
    /**
     * added here to satisfy the watchdog
     */ 
    @Override
    public void simulationPeriodic(){
    }

    public void runningLED() {
        if ((countLED % 25) == 0) {
            if (prevLEDValue) {
                RobotContainer.controlpanel.setRunningLED(false);
                prevLEDValue = false;
            } else {
                RobotContainer.controlpanel.setRunningLED(true);
                prevLEDValue = true;
            }
            countLED = 1;
        } else {
            countLED += 1;
        }
    }
}
