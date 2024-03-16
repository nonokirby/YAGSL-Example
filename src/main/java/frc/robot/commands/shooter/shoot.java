package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class shoot extends InstantCommand{
    public shoot(){
        addRequirements(RobotContainer.shooter);
    }

    public void execute() {
        RobotContainer.shooter.flywheel(1);
    }

    public void end(boolean interrupted) {
        RobotContainer.shooter.feed(1);
    }
    
}
