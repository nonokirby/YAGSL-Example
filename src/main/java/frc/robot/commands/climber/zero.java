package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class zero extends Command{
    
    public zero(){
        addRequirements(RobotContainer.climber);
    }
    public void execute(){
        RobotContainer.climber.zero();
    }
    public void end(Boolean interrupted){
        if (interrupted == true){
          System.out.println("Zeroing Climber Failed");
        }
    }
}
