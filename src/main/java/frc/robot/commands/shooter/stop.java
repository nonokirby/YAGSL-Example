package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class stop extends InstantCommand{
    public stop(){
        addRequirements(RobotContainer.shooter);
    }

    public void execute() {
        RobotContainer.shooter.feed(0);
        RobotContainer.shooter.flywheel(0);
    }

    public void end(boolean interrupted) {

    }


}
