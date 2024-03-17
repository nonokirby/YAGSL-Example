package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
public class shooter extends SubsystemBase {
  
  CANSparkMax mIntake = new CANSparkMax(intake.id, intake.neo);
  CANSparkMax mFlywheelL = new CANSparkMax(flywheelL.id, flywheelL.neo);
  CANSparkMax mFlywheelR = new CANSparkMax(flywheelR.id, flywheelR.neo);
  CANSparkMax mFeederL = new CANSparkMax(feederL.id, feederL.neo);
  CANSparkMax mFeederR = new CANSparkMax(feederR.id, feederR.neo);
  double speed;
  double speedL;
  double speedR;

    public void intake(double input_speed){
      double speed = input_speed * intake.power;
      mIntake.set(speed);
    }

    public void flywheel(double input_speed){
      double speedL = input_speed * flywheelL.power;
      double speedR = input_speed * flywheelR.power;
      mFlywheelL.set(speedL);
      mFlywheelR.set(speedR);
    }
    
    public void feed(double input_speed){
      double speedL = input_speed * feederL.power;
      double speedR = input_speed * feederR.power;
      mFeederL.set(speedL);
      mFeederR.set(speedR);
    }

    public void shoot(double feedRate, double flywheelSpeed){
      double flywheelSpeedL = feedRate * flywheelL.power;
      double flywheelSpeedR = feedRate * flywheelR.power;
      double feedSpeedL = feedRate * feederL.power;
      double feedSpeedR = feedRate * feederR.power;
      mFlywheelL.set(flywheelSpeedL);
      mFlywheelR.set(flywheelSpeedR);
      mFeederL.set(feedSpeedL);
      mFeederR.set(feedSpeedR);    
    } 

    public void intakeboth(double feedRate, double intakerate){
      double intakeSpeed = feedRate * intake.power;
      double feedSpeedL = feedRate * feederL.power;
      double feedSpeedR = feedRate * feederR.power;
      mIntake.set(intakeSpeed);
      mFeederL.set(feedSpeedL);
      mFeederR.set(feedSpeedR);    
    } 

   /* public void periodic(){
        double flywheelSpeedL = RobotContainer.shooterXbox.getLeftTriggerAxis() * flywheelL.power;
        double flywheelSpeedR = RobotContainer.shooterXbox.getLeftTriggerAxis() * flywheelR.power;
        mFlywheelL.set(flywheelSpeedL);
        mFlywheelR.set(flywheelSpeedR);
      }*/
    }

    

