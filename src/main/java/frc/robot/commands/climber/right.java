package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class right extends Command{
    
    public right(){
        addRequirements(RobotContainer.climber);
    }
    public void execute(){
        RobotContainer.climber.setInd(1,1);
    }
    public void end(){
    }
}
