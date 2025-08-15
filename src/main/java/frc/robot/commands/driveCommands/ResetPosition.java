package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class ResetPosition extends CommandBase {
    //! Subsystem Initialisation
    private static final DriveTrain drivetrain = RobotContainer.drivetrain;

    //! Variable Initialisation
    private double setpointX;
    private double setpointY;
    private double setpointTh;
    private boolean IMU;

    /**
     * <h2> Command to Reset Coordinate Position Elements of Robot </h2>
     * @param setpointX
     * @param setpointY
     * @param setpointTh
     * @param IMU (Set to true to reset IMU)
     */
    public ResetPosition(double setpointX, double setpointY, double setpointTh, boolean IMU) {
        //! Parameter Declaration
        this.setpointX = setpointX;
        this.setpointY = setpointY;
        this.setpointTh = setpointTh;
        this.IMU = IMU;
    }
    public ResetPosition(){}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.stopMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (IMU) {
            drivetrain.ZeroIMU();
        // }
        // Pose2d TempPose = new Pose2d(setpointX, setpointY, new Rotation2d(setpointTh));
        // drivetrain.setPose(TempPose);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
