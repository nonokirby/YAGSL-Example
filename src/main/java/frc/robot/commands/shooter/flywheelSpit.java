package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class flywheelSpit extends Command{
    public flywheelSpit(){
        addRequirements(RobotContainer.shooter);
    }

    public void execute() {
        RobotContainer.shooter.flywheel(-1);
        RobotContainer.shooter.intakeboth(-1, 0);
    }

    public void end(boolean interrupted) {
        //RobotContainer.shooter.triggerFlywheel(false);
        //System.out.print("end");
        RobotContainer.shooter.flywheel(0);
        RobotContainer.shooter.intakeboth(0, 0);
    }


}
