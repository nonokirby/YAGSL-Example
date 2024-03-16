package frc.robot.commands.flap;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class lower extends InstantCommand {
    
    public lower(){
        addRequirements(RobotContainer.flap);
    }
    public void execute(){
        RobotContainer.flap.set(-1);
    }
    public void end(){
        //RobotContainer.climber.climb(0);
    }
    
}
