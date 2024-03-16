package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class down extends InstantCommand {
    
    public down(){
        addRequirements(RobotContainer.climber);
    }
    public void execute(){
        RobotContainer.climber.set(-1);
    }
    public void end(){
        RobotContainer.climber.set(0);
    }
    
}
