package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class up extends InstantCommand{
    
    public up(){
        addRequirements(RobotContainer.climber);
    }
    public void execute(){
        RobotContainer.climber.set(1);
    }
    public void end(){
    }
    public void initialize(){
        RobotContainer.climber.setup();
    }
    
}
