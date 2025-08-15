package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.autonomous.AutoFlow;
import frc.robot.commands.defaultCommands.TeleOp;
import frc.robot.subsystems.ControlPanel;
// import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gamepad;
// import frc.robot.subsystems.Lidar360;
import frc.robot.subsystems.Lidar360;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static DriveTrain drivetrain;
    public static Gamepad gamepad;
    public static Lidar360 lidar;
    public static ControlPanel controlpanel;

    public static SendableChooser<String> autoChooser;
    public static Map<String, AutoCommand> autoMode = new HashMap<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Create new instances
        drivetrain = new DriveTrain();
        gamepad = new Gamepad();
        lidar = new Lidar360();
        controlpanel = new ControlPanel();

        // Set default command for the drive train subsystem
        drivetrain.setDefaultCommand(new TeleOp());
    }

    public Command getAutonomousCommand()
    {
        String mode = autoChooser.getSelected();
        return autoMode.getOrDefault(mode, new AutoFlow());
    }
}
