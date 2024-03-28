package frc.robot.commands.flap;

import javax.swing.plaf.basic.BasicTreeUI.TreeCancelEditingAction;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class flapszero extends Command{
    
    public flapszero(){
        addRequirements(RobotContainer.flap);
    }
    public void execute(){
        RobotContainer.flap.set(-1);}
    public void end(){
        RobotContainer.flap.zero();
        RobotContainer.flap.set(0);
        //RobotContainer.climber.climb(0);
    }
    public boolean isFinished(){
        return RobotContainer.flap.sw.get();
    }
    
    
}
