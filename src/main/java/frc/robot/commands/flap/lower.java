package frc.robot.commands.flap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class lower extends Command{
    
    public lower(){
        addRequirements(RobotContainer.flap);
    }
    public void execute(){
        RobotContainer.flap.setpoint(0);
    }
    public void end(){
        //RobotContainer.climber.climb(0);
    }
    
}
