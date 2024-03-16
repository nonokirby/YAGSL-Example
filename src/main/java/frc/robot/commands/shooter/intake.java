package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class intake extends InstantCommand{
    public intake(){
        addRequirements(RobotContainer.shooter);
    }

    public void execute() {
        RobotContainer.shooter.intake(1);
    }

    public void end(boolean interrupted) {
        RobotContainer.shooter.intake(0);
    }


}
