package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class stop extends Command{
    public stop(){
        addRequirements(RobotContainer.shooter);
    }

    public void execute() {
    }

    public void end(boolean interrupted) {
    }


}
