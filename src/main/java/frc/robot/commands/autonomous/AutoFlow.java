package frc.robot.commands.autonomous;

import frc.robot.commands.driveCommands.ObsAvoid;
import frc.robot.commands.driveCommands.ResetPosition;


public class AutoFlow extends AutoCommand {

private static double xDist = 1; 
private static double yDist = 0; 

/** Constructor */
public AutoFlow() 
{
    super(

        new ResetPosition(),

        new ObsAvoid(xDist*1.08, yDist*1.08, 0)
        // new ObsAvoid(xDist, yDist, 0, false)
        // new StateMachine(),
        // new Drive().withTimeout(3)
        
    );
}
}
