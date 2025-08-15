package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lidar360;

public class ObsAvoid extends CommandBase {
    public static final DriveTrain drivetrain = RobotContainer.drivetrain;
    public static final Lidar360 lidar = RobotContainer.lidar;

    private enum OD {
        Initial,
        InitialRotatePID,
        RotateToGoal,
        MoveBack,
        SlowDownBack,
        ObsAvoidance,
        MoveToFinalGoal,
        FinalRotate,
        SlowDown,
        StuckPID,
        StuckRotate
    }

    public static final class Pose2d { // me when no struct 🥺
        public double x = 0, y = 0, th = 0, gyroAngle = 0;        
        public Pose2d(){}; // why do i need to put this lol
        public Pose2d(double x, double y, double th) {this.x=x;this.y=y;this.th=th;}
        public Pose2d(double x, double y, double th, double gyroAngle) {this(x, y, th);this.gyroAngle=gyroAngle;}
    }

    public Pose2d setpoints, velocity; 

    private OD state = OD.Initial;

    private double angle, status, startTime, timeTaken, deltaX, deltaY, rotation;

    private double reactivity = 0;//0.5;//0.8; // Determines how fast the robot rotates
    private double forwardSpd = 0;
    private boolean start = true;
    private boolean endCondition = false;

    private double initialRotation;

    public SlewRateLimiter filterTH = new SlewRateLimiter(5);//25, 1 // Decrease slew rate to improve smooth movement and damp jerky movement
    public SlewRateLimiter filterX = new SlewRateLimiter(10);

    public ObsAvoid(double goalX, double goalY, double goalTh) {
        addRequirements(drivetrain);
        addRequirements(lidar);

        this.velocity = new Pose2d();
        this.setpoints = new Pose2d();

        this.setpoints.x = goalX;
        this.setpoints.y = goalY;
        this.setpoints.th = Math.toRadians(goalTh);
    }

    /**
     * Runs once before execute
     */
    @Override
    public void initialize() {
        System.out.println("ObstacleAvoid started"); 

        if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.FourWheel) {
            this.reactivity = 0.8;
            this.forwardSpd = 0.3;
        }
        else if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.TwoWheel) {
            this.reactivity = 0.4;
            this.forwardSpd = 0.3;
        }
        else if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.Mecanum) {
            this.reactivity = 0.8;
            this.forwardSpd = 0.25;
        }
        else if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.SixWheel) {
            this.reactivity = 0.3;
            this.forwardSpd = 0.3;
        }
        else if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.XDrive) {
            this.reactivity = 0.45;//0.52
            this.forwardSpd = 0.25;
        }
        else if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.KiwiDrive) {
            this.reactivity = 0.85;
            this.forwardSpd = 0.35;
        }

        state = OD.ObsAvoidance;

        //Starts filter at 0
        filterX.calculate(0);
        filterTH.calculate(0);


        initialRotation = drivetrain.getYaw(false);
        lidar.resetNavigationState();

    }

    /**
     * Called continously until command is ended
     */
    @Override
    public void execute() {

        switch(state) {
            case Initial:
                // System.out.println("WARNING: (UnexpectedBehaviour) ObstacleDrive is at state Initial");
                if (lidar.frontClear(0.3)) {
                    state = OD.ObsAvoidance;
                    System.out.println("Front clear");
                    lidar.resetNavigationState();
                }
                else {
                    state = OD.MoveBack;
                    startTime = System.currentTimeMillis();
                }
                break;

            case MoveBack:
                velocity.x = -0.25;
                velocity.y = 0; velocity.th = 0;
                velocity.gyroAngle = 0.0;
                timeTaken = (System.currentTimeMillis() - startTime) / 1000;
                if (timeTaken > 1.5) {
                    state = OD.SlowDownBack;
                }
                break;
            
            case SlowDownBack:
                velocity.x *= 0.95f;
                velocity.y = 0; velocity.th = 0;
                velocity.gyroAngle = 0.0;
                if (Math.abs(velocity.x) < 0.1) {
                    velocity.x = 0;
                    state = OD.ObsAvoidance;
                }
                break;

            case ObsAvoidance:

                break;

            case StuckPID:

                // setInitialPID(currentPose.x, currentPose.y, normaliseAngle(currentPose.th));
                // double newTH = Math.abs(setpoints.th) == Math.PI ? (normaliseAngle(currentPose.th) > 0 ? (setpoints.th > 0 ? setpoints.th : setpoints.th * -1) : (setpoints.th < 0 ? setpoints.th : setpoints.th * -1)) : setpoints.th;
                // setGoalPID(setpoints.x, setpoints.y, normaliseAngle(newTH));
                deltaX = setpoints.x;
                deltaY = setpoints.y;
                rotation = Math.atan2(deltaY, deltaX);
                drivetrain.rotation_controller.setTolerance(Math.toRadians(1));
                drivetrain.rotation_controller.reset(normaliseAngle(drivetrain.getYaw(false)), 0.0);
                // drivetrain.rotation_controller.setGoal(rotation); //! Robot faces to the target
                drivetrain.rotation_controller.setGoal(initialRotation); //! Robot faces the front
                drivetrain.rotation_controller.calculate(drivetrain.getYaw(false));

                state = OD.StuckRotate;
                startTime = System.currentTimeMillis();


                // System.out.println("Inital pose: ");
                // System.out.println(normaliseAngle(currentPose.th));
                // System.out.println("Target pose: ");
                // System.out.println(setpoints.th);
                
                break;
                
            case StuckRotate:
                velocity.x = 0;
                velocity.y = 0;
                velocity.th = drivetrain.rotation_controller.calculate(normaliseAngle(drivetrain.getYaw(false)));
                // velocity.gyroAngle = getGyroAngle();
                timeTaken = (System.currentTimeMillis() - startTime) / 1000;
                if (drivetrain.rotation_controller.atGoal() || timeTaken > 10) {
                    velocity.x = 0; velocity.y = 0; velocity.th = 0;
                    state = OD.Initial;
                }
                break;
            
            default:
                velocity.x = 0;
                velocity.th = 0;
                System.out.println("ERROR: ObsAvoid switch case error! Invalid state!");
                break;

        }


        if (Constants.CURRENTROBOT == Constants.ROBOTTYPE.KiwiDrive) {
            drivetrain.newKiwiMotorControl(velocity.x, velocity.y, velocity.th, 0);
        }
        


    }

    // Returns angles between -pi to pi (Radians)
    public double normaliseAngle(double radians) { //!This should be correct :/
        double degrees = Math.toDegrees(radians);
        while (degrees > 180) {
            degrees -= 360;
        } 
        while (degrees < -180) {
            degrees += 360;
        }
        
        return Math.toRadians(degrees);
    }

    /**
     * Called when the command is told to end
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMotors();
        drivetrain.rotation_controller.reset(0.0, 0.0);
        System.out.println("\n\n--! ENDING COMMAND: " + this.getClass().getName() + "\n\n");
    }

    /**
     * Creates the end condition for the command
     */
    @Override
    public boolean isFinished() {
        return endCondition;
    }
}