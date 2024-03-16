package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class flywheel extends InstantCommand{
    public flywheel(){
        addRequirements(RobotContainer.shooter);
    }

    public void execute() {
        RobotContainer.shooter.flywheel(.7);
    }

    public void end(boolean interrupted) {
        //RobotContainer.shooter.triggerFlywheel(false);
        //System.out.print("end");
        RobotContainer.shooter.flywheel(0);
    }


}
