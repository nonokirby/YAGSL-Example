package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class eject extends Command{
    public eject(){
        addRequirements(RobotContainer.shooter);
    }

    public void execute() {
        RobotContainer.shooter.intakeboth(-1,-1);
    }

    public void end(boolean interrupted) {
        RobotContainer.shooter.intakeboth(0,0);
    }
    
}
