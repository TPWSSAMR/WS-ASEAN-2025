package frc.robot.commands.defaultCommands;

import com.studica.frc.Lidar;

// Library Imports
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

// File Imports
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gamepad;
import frc.robot.subsystems.Lidar360;

public class TeleOp extends CommandBase {
    // ! Subsystem Initialisation
    private static final DriveTrain drivetrain = RobotContainer.drivetrain;
    private static final Lidar360 lidar = RobotContainer.lidar;
    private static final Gamepad gamepad = RobotContainer.gamepad;

    //! TeleOp Initialisation
    private double[] cmd_vel = { 0.0, 0.0 ,0.0};
    private double heading = 0.0;

    private boolean debounce = false;
    private double stampedTime;

    public TeleOp() {
        addRequirements(drivetrain);
        addRequirements(lidar);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // This sets all motor velocities to 0
        cmd_vel[0] = 0.0;
        cmd_vel[1] = 0.0;
    }

    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //! Initialisation Button
        if(gamepad.getButtonStart())
        {
            drivetrain.initialise();
        } else if (gamepad.getButtonA()) {

            if (!debounce) {
                lidar.debugger(lidar.simpleOffset(270, -0.095));
                stampedTime = System.currentTimeMillis();
                debounce = true;
            }
            
        }

        if (System.currentTimeMillis() - stampedTime >= 2000) {
            debounce = false;
        } 

        //! Movement Control
        cmd_vel[0] = -gamepad.getJoystickLeftY();
        cmd_vel[1] = -gamepad.getJoystickLeftX();
        cmd_vel[2] = -gamepad.getJoystickRightX();
        // cmd_vel[0] = -gamepad.getJoystickLeftY();
        // cmd_vel[1] = -gamepad.getJoystickRightX();
        heading = drivetrain.getYaw(false);
        // System.out.println("VX: " + cmd_vel[0]);
        // System.out.println("VY: " + cmd_vel[1]);
        // System.out.println("VTH: " + cmd_vel[2]);
        // drivetrain.setMotorsSpeed(cmd_vel[0]+cmd_vel[1], cmd_vel[0]-cmd_vel[1], cmd_vel[0]+cmd_vel[1], cmd_vel[0]-cmd_vel[1]);
        
        if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.KiwiDrive) {
            drivetrain.kiwiMotorControl(cmd_vel[0], cmd_vel[1], cmd_vel[2] * 2 * Math.PI, heading);
            // drivetrain.kiwiMotorControl(cmd_vel[0], cmd_vel[1], cmd_vel[2], heading);
        }



        // cmd_vel[0] = -filterX.calculate(gamepad.getJoystickLeftY());
        // cmd_vel[1] = -filterY.calculate(gamepad.getJoystickLeftX());
        // cmd_vel[2] = -filterTh.calculate(gamepad.getJoystickRightX());

        //Original for omni
        // cmd_vel[0] = -filterX.calculate(gamepad.getJoystickLeftY());
        // cmd_vel[1] = -filterY.calculate(gamepad.getJoystickLeftX());
        // cmd_vel[2] = -filterTh.calculate(gamepad.getJoystickRightX() * 2 * Math.PI);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
