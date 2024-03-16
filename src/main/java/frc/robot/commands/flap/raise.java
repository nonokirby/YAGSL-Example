package frc.robot.commands.flap;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class raise extends InstantCommand {
    
    public raise(){
        addRequirements(RobotContainer.flap);
    }
    public void execute(){
        RobotContainer.flap.set(1);
    }
    public void end(){
        //RobotContainer.climber.climb(0);
    }
    
}
