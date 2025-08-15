package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.Lidar;
import com.studica.frc.TitanQuad;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Odom;

public class DriveTrain extends SubsystemBase
{
    // Debug Flag
    private boolean debug = true;

    private TitanQuad frontLeftMotor;
    private TitanQuad backLeftMotor;
    private TitanQuad frontRightMotor;
    private TitanQuad backRightMotor;
    private final double[] desiredMotorVel = new double[4];

    private final Odom odom;

    private AHRS imu;

    private double loopTime;
    private long prevTime, currTime;
   
    private double sx, sy, dth;

    private Lidar lidar;
    private Lidar.ScanData scanData;
    public boolean scanning = true; // flag to prevent updating when not scanning

    double denomantor = 0;

    public final ProfiledPIDController rotation_controller = new ProfiledPIDController(3, 0.8, 0.1, new TrapezoidProfile.Constraints( 1, 1 ), 0.02);
    /**
     * DriveTrain Constructor
     */
    public DriveTrain()
    {

        frontLeftMotor = new TitanQuad(Constants.TITAN_ID, Constants.FRONT_LEFT_MOTOR);
        backLeftMotor = new TitanQuad(Constants.TITAN_ID, Constants.BACK_LEFT_MOTOR);
        frontRightMotor = new TitanQuad(Constants.TITAN_ID, Constants.FRONT_RIGHT_MOTOR);
        backRightMotor = new TitanQuad(Constants.TITAN_ID, Constants.BACK_RIGHT_MOTOR);

        Timer.delay(1.0); // Wait 1s for Titan to configure

        //! May need to change

        if (Constants.CURRENTROBOT != Constants.ROBOTTYPE.KiwiDrive) {

        }
        else {

        }

        imu = new AHRS(SPI.Port.kMXP);

        lidar = new Lidar(Lidar.Port.kUSB2);

        imu.calibrate();
        while (imu.isCalibrating()) {/*I love java*/}
        imu.zeroYaw();

        odom = new Odom();

        prevTime = System.nanoTime();
    }

    /**
     * Reset Yaw
     */
    public void resetYaw()
    {
        imu.zeroYaw();
    }

    /**
     * Start the lidar if it is stopped.
     */
    public void startLidar()
    {
        lidar.start();
        scanning = true;
    }

    /**
     * Stop the lidar if required. This will ease the load on CPU.
     */
    public void stopLidar()
    {
        lidar.stop();
        scanning = false;
    }


    /**
     * <h2> Sub-Command to retrieve the current yaw of the robot from the IMU sensor </h2>
     * @param continuous (Set to False to receive a reading based on true north, ranging from pi to -pi)
     * @return (Rotational Heading of Robot in Radians)
     */
    public double getYaw(boolean continuous)
    {
        // return imu.getYaw();
        return continuous == true ? -Math.toRadians(imu.getAngle()) : -Math.toRadians(imu.getYaw());

    }

    /**
     * Zero the yaw of the internal IMU.
     */
    public void ZeroIMU()
    {
        imu.zeroYaw();
    }

    public void initialise() {
        imu.zeroYaw();
        odom.resetPose();
        imu.resetDisplacement();
    }

    public void stopMotors() {
        frontRightMotor.stopMotor();
        frontLeftMotor.stopMotor();
        backRightMotor.stopMotor();
        backLeftMotor.stopMotor();
        desiredMotorVel[0] = 0.0;
        desiredMotorVel[1] = 0.0;
        desiredMotorVel[2] = 0.0;
        desiredMotorVel[3] = 0.0;
    }

    public void setMotorsSpeed(double frontLeft, double frontRight, double backLeft, double backRight) {
        desiredMotorVel[0] = frontLeft;
        desiredMotorVel[1] = frontRight;
        desiredMotorVel[2] = backLeft;
        desiredMotorVel[3] = backRight;
    }


    public void newKiwiMotorControl(double vX, double vY, double vTH, double headingRadians) {

    }

    public void kiwiMotorControl(double vX, double vY, double vTH, double headingRadians) {

    }

    public Pose2d getPose() {
        return odom.getPose();
    }

    /**
     * Everything in this method runs once every robot loop a.k.a 20ms
     */
    @Override
    public void periodic() {
        frontLeftMotor.set(desiredMotorVel[0]);
        frontRightMotor.set(desiredMotorVel[1]);
        backLeftMotor.set(desiredMotorVel[2]);
        backRightMotor.set(desiredMotorVel[3]);

        // Loop Duration Calculation
        currTime = System.nanoTime();
        loopTime = (currTime - prevTime) / 1e9d;   // This calculates the loop duration
        prevTime = currTime;                       // This saves the current time

        // sx = (0.5 * imu.getRawAccelX()*9.81 * Math.pow(loopTime, 2));
        // sy = (0.5 * imu.getRawAccelY()*9.81 * Math.pow(loopTime, 2));
        sx = imu.getVelocityX() * loopTime;
        sy = imu.getVelocityY() * loopTime;
        
        dth = Math.toRadians(imu.getRate()) * loopTime;

        odom.update(sx, sy, dth);

        if (Constants.DEBUG_DRIVETRAIN) {

            // Robot Odometry
            SmartDashboard.putNumber(" Odom - X:", getPose().getTranslation().getX());
            SmartDashboard.putNumber(" Odom - Y:", getPose().getTranslation().getY());
            SmartDashboard.putNumber(" Odom - ThD:", Math.toDegrees(getPose().getRotation().getDegrees()));
            SmartDashboard.putNumber(" Odom - ThR:", Math.toDegrees(getPose().getRotation().getRadians()));
            SmartDashboard.putNumber(" Position [IMU S]:", Math.toDegrees(getYaw(false)));
            SmartDashboard.putNumber(" Position [IMU C]:", Math.toDegrees(getYaw(true)));
            
            SmartDashboard.putNumber(" Position [IMU X]:", imu.getDisplacementX());
            SmartDashboard.putNumber(" Position [IMU Y]:", imu.getDisplacementY());
            SmartDashboard.putNumber(" Position [IMU Z]:", imu.getDisplacementZ());


            // SmartDashboard.putNumber(" Position [AX G]:", imu.getWorldLinearAccelX());
            // SmartDashboard.putNumber(" Position [AY G]:", imu.getWorldLinearAccelY());
            // SmartDashboard.putNumber(" Position [AZ G]:", imu.getWorldLinearAccelZ());

            SmartDashboard.putNumber(" Position [AX (m^2s^-2]:", (imu.getWorldLinearAccelX()*9.81));
            SmartDashboard.putNumber(" Position [AY (m^2s^-2]:", (imu.getWorldLinearAccelY()*9.81));
            SmartDashboard.putNumber(" Position [AZ (m^2s^-2]:", (imu.getWorldLinearAccelZ()*9.81));





           
        }

    }
}