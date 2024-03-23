package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class down extends Command{
    
    public down(){
        addRequirements(RobotContainer.climber);
    }
    public void execute(){
        RobotContainer.climber.goSet(0);
        System.out.println("Climber Down");
    }
    public void end(){
        
    }
}
