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
        RobotContainer.flap.set(-.75);
        }
    public void end(){
        RobotContainer.flap.zero();
        RobotContainer.flap.set(0);
    }
    public boolean isFinished(){
        return RobotContainer.flap.sw.get();
    }
    
}
