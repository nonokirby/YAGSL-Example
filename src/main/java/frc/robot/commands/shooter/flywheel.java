package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class flywheel extends InstantCommand{
    public flywheel(){
        addRequirements(RobotContainer.shooter);
    }

    public void execute() {
        //RobotContainer.shooter.triggerFlywheel(true);
        System.out.print("execute");
    }

    public void end(boolean interrupted) {
        //RobotContainer.shooter.triggerFlywheel(false);
        System.out.print("end");
    }


}
