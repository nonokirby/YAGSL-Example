package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class feed extends Command{
    public feed(){
        addRequirements(RobotContainer.shooter);
    }

    public void execute() {
        RobotContainer.shooter.feed(1);
        RobotContainer.shooter.flywheel(1);
        
    }

    public void end(boolean interrupted) {
        RobotContainer.shooter.feed(0);
        RobotContainer.shooter.flywheel(0);
    }
    
    


}
