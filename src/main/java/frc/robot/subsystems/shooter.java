package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      public void periodic(){
        // FlywheelL
        SmartDashboard.putNumber("Neo_Encoder_FlywheelL_Pos", mFlywheelL.getEncoder().getPosition());
        SmartDashboard.putNumber("Neo_Encoder_FlywheelL_Vel", mFlywheelL.getEncoder().getVelocity());
        SmartDashboard.putNumber("Controller_FlywheelL_Current", mFlywheelL.getOutputCurrent());
        SmartDashboard.putNumber("Controller_FlywheelL_Temp", mFlywheelL.getMotorTemperature());
        SmartDashboard.putNumber("Controller_FlywheelL_BusVoltage", mFlywheelL.getBusVoltage());
        SmartDashboard.putNumber("Controller_FlywheelL_AppliedOutput", mFlywheelL.getAppliedOutput());
        SmartDashboard.putNumber("Controller_FlywheelL_DeviceId", mFlywheelL.getDeviceId());

        // FlywheelR
        SmartDashboard.putNumber("Neo_Encoder_FlywheelR_Pos", mFlywheelR.getEncoder().getPosition());
        SmartDashboard.putNumber("Neo_Encoder_FlywheelR_Vel", mFlywheelR.getEncoder().getVelocity());
        SmartDashboard.putNumber("Controller_FlywheelR_Current", mFlywheelR.getOutputCurrent());
        SmartDashboard.putNumber("Controller_FlywheelR_Temp", mFlywheelR.getMotorTemperature());
        SmartDashboard.putNumber("Controller_FlywheelR_BusVoltage", mFlywheelR.getBusVoltage());
        SmartDashboard.putNumber("Controller_FlywheelR_AppliedOutput", mFlywheelR.getAppliedOutput());
        SmartDashboard.putNumber("Controller_FlywheelR_DeviceId", mFlywheelR.getDeviceId());

        // mIntake
        SmartDashboard.putNumber("Neo_Encoder_mIntake_Pos", mIntake.getEncoder().getPosition());
        SmartDashboard.putNumber("Neo_Encoder_mIntake_Vel", mIntake.getEncoder().getVelocity());
        SmartDashboard.putNumber("Controller_mIntake_Current", mIntake.getOutputCurrent());
        SmartDashboard.putNumber("Controller_mIntake_Temp", mIntake.getMotorTemperature());
        SmartDashboard.putNumber("Controller_mIntake_BusVoltage", mIntake.getBusVoltage());
        SmartDashboard.putNumber("Controller_mIntake_AppliedOutput", mIntake.getAppliedOutput());
        SmartDashboard.putNumber("Controller_mIntake_DeviceId", mIntake.getDeviceId());

        // FeederL
        SmartDashboard.putNumber("Neo_Encoder_FeederL_Pos", mFeederL.getEncoder().getPosition());
        SmartDashboard.putNumber("Neo_Encoder_FeederL_Vel", mFeederL.getEncoder().getVelocity());
        SmartDashboard.putNumber("Controller_FeederL_Current", mFeederL.getOutputCurrent());
        SmartDashboard.putNumber("Controller_FeederL_Temp", mFeederL.getMotorTemperature());
        SmartDashboard.putNumber("Controller_FeederL_BusVoltage", mFeederL.getBusVoltage());
        SmartDashboard.putNumber("Controller_FeederL_DeviceId", mFeederL.getDeviceId());

        // FeederR
        SmartDashboard.putNumber("Neo_Encoder_FeederR_Pos", mFeederR.getEncoder().getPosition());
        SmartDashboard.putNumber("Neo_Encoder_FeederR_Vel", mFeederR.getEncoder().getVelocity());
        SmartDashboard.putNumber("Controller_FeederR_Current", mFeederR.getOutputCurrent());
        SmartDashboard.putNumber("Controller_FeederR_Temp", mFeederR.getMotorTemperature());
        SmartDashboard.putNumber("Controller_FeederR_BusVoltage", mFeederR.getBusVoltage());
        SmartDashboard.putNumber("Controller_FeederR_AppliedOutput", mFeederR.getAppliedOutput());
        SmartDashboard.putNumber("Controller_FeederR_DeviceId", mFeederR.getDeviceId());

        SmartDashboard.putNumber("test", mFeederL.getFaults());
        mFeederL.get();
        mFeederL.getFirmwareString();
      }    
   }