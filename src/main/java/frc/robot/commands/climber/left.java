package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class left extends Command{
    
    public left(){
        addRequirements(RobotContainer.climber);
    }
    public void execute(){
        RobotContainer.climber.goSet(8000);
    }
    public void end(){
    }
    
}