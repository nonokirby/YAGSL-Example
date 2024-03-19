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
        SmartDashboard.putNumber("Neo_Encoder_FeederL_Pos", mFeederL.getEncoder().getPosition());
        SmartDashboard.putNumber("Neo_Encoder_FeederL_Vel", mFeederL.getEncoder().getVelocity());
        SmartDashboard.putNumber("Controller_FeederL_Current", mFeederL.getOutputCurrent());
        SmartDashboard.putNumber("Controller_FeederL_Temp", mFeederL.getMotorTemperature());
        SmartDashboard.putNumber("Controller_FeederL_BusVoltage", mFeederL.getBusVoltage());
        SmartDashboard.putNumber("Controller_FeederL_AppliedOutput", mFeederL.getAppliedOutput());
        SmartDashboard.putNumber("Controller_FeederL_DeviceId", mFeederL.getDeviceId());
        SmartDashboard.putNumber("Controller_FeederL_Faults", mFeederL.getFaults());
        SmartDashboard.putNumber("Controller_FeederL_Get", mFeederL.get());
        SmartDashboard.putString("Controller_FeederL_FirmwareString", mFeederL.getFirmwareString());
        SmartDashboard.putString("Controller_FeederL_IdleMode", mFeederL.getIdleMode().toString());
        SmartDashboard.putString("Controller_FeederL_SerialNumber", mFeederL.getSerialNumber().toString());
        SmartDashboard.putNumber("Controller_FeederL_StickyFaults", mFeederL.getStickyFaults());
        SmartDashboard.putNumber("Controller_FeederL_VoltageCompensationNominalVoltage", mFeederL.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("Controller_FeederL_MotorType", mFeederL.getMotorType().toString());

        // FeederR
        SmartDashboard.putNumber("Neo_Encoder_FeederR_Pos", mFeederR.getEncoder().getPosition());
        SmartDashboard.putNumber("Neo_Encoder_FeederR_Vel", mFeederR.getEncoder().getVelocity());
        SmartDashboard.putNumber("Controller_FeederR_Current", mFeederR.getOutputCurrent());
        SmartDashboard.putNumber("Controller_FeederR_Temp", mFeederR.getMotorTemperature());
        SmartDashboard.putNumber("Controller_FeederR_BusVoltage", mFeederR.getBusVoltage());
        SmartDashboard.putNumber("Controller_FeederR_AppliedOutput", mFeederR.getAppliedOutput());
        SmartDashboard.putNumber("Controller_FeederR_DeviceId", mFeederR.getDeviceId());
        SmartDashboard.putNumber("Controller_FeederR_Faults", mFeederR.getFaults());
        SmartDashboard.putNumber("Controller_FeederR_Get", mFeederR.get());
        SmartDashboard.putString("Controller_FeederR_FirmwareString", mFeederR.getFirmwareString());
        SmartDashboard.putString("Controller_FeederR_IdleMode", mFeederR.getIdleMode().toString());
        SmartDashboard.putString("Controller_FeederR_SerialNumber", mFeederR.getSerialNumber().toString());
        SmartDashboard.putNumber("Controller_FeederR_StickyFaults", mFeederR.getStickyFaults());
        SmartDashboard.putNumber("Controller_FeederR_VoltageCompensationNominalVoltage", mFeederR.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("Controller_FeederR_MotorType", mFeederR.getMotorType().toString());

        // Intake
        SmartDashboard.putNumber("Neo_Encoder_Intake_Pos", mIntake.getEncoder().getPosition());
        SmartDashboard.putNumber("Neo_Encoder_Intake_Vel", mIntake.getEncoder().getVelocity());
        SmartDashboard.putNumber("Controller_Intake_Current", mIntake.getOutputCurrent());
        SmartDashboard.putNumber("Controller_Intake_Temp", mIntake.getMotorTemperature());
        SmartDashboard.putNumber("Controller_Intake_BusVoltage", mIntake.getBusVoltage());
        SmartDashboard.putNumber("Controller_Intake_AppliedOutput", mIntake.getAppliedOutput());
        SmartDashboard.putNumber("Controller_Intake_DeviceId", mIntake.getDeviceId());
        SmartDashboard.putNumber("Controller_Intake_Faults", mIntake.getFaults());
        SmartDashboard.putNumber("Controller_Intake_Get", mIntake.get());
        SmartDashboard.putString("Controller_Intake_FirmwareString", mIntake.getFirmwareString());
        SmartDashboard.putString("Controller_Intake_IdleMode", mIntake.getIdleMode().toString());
        SmartDashboard.putString("Controller_Intake_SerialNumber", mIntake.getSerialNumber().toString());
        SmartDashboard.putNumber("Controller_Intake_StickyFaults", mIntake.getStickyFaults());
        SmartDashboard.putNumber("Controller_Intake_VoltageCompensationNominalVoltage", mIntake.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("Controller_Intake_MotorType", mIntake.getMotorType().toString());

        // FlywheelR
        SmartDashboard.putNumber("Neo_Encoder_FlywheelR_Pos", mFlywheelR.getEncoder().getPosition());
        SmartDashboard.putNumber("Neo_Encoder_FlywheelR_Vel", mFlywheelR.getEncoder().getVelocity());
        SmartDashboard.putNumber("Controller_FlywheelR_Current", mFlywheelR.getOutputCurrent());
        SmartDashboard.putNumber("Controller_FlywheelR_Temp", mFlywheelR.getMotorTemperature());
        SmartDashboard.putNumber("Controller_FlywheelR_BusVoltage", mFlywheelR.getBusVoltage());
        SmartDashboard.putNumber("Controller_FlywheelR_AppliedOutput", mFlywheelR.getAppliedOutput());
        SmartDashboard.putNumber("Controller_FlywheelR_DeviceId", mFlywheelR.getDeviceId());
        SmartDashboard.putNumber("Controller_FlywheelR_Faults", mFlywheelR.getFaults());
        SmartDashboard.putNumber("Controller_FlywheelR_Get", mFlywheelR.get());
        SmartDashboard.putString("Controller_FlywheelR_FirmwareString", mFlywheelR.getFirmwareString());
        SmartDashboard.putString("Controller_FlywheelR_IdleMode", mFlywheelR.getIdleMode().toString());
        SmartDashboard.putString("Controller_FlywheelR_SerialNumber", mFlywheelR.getSerialNumber().toString());
        SmartDashboard.putNumber("Controller_FlywheelR_StickyFaults", mFlywheelR.getStickyFaults());
        SmartDashboard.putNumber("Controller_FlywheelR_VoltageCompensationNominalVoltage", mFlywheelR.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("Controller_FlywheelR_MotorType", mFlywheelR.getMotorType().toString());

        
        // FlywheelL
        SmartDashboard.putNumber("Neo_Encoder_FlywheelL_Pos", mFlywheelL.getEncoder().getPosition());
        SmartDashboard.putNumber("Neo_Encoder_FlywheelL_Vel", mFlywheelL.getEncoder().getVelocity());
        SmartDashboard.putNumber("Controller_FlywheelL_Current", mFlywheelL.getOutputCurrent());
        SmartDashboard.putNumber("Controller_FlywheelL_Temp", mFlywheelL.getMotorTemperature());
        SmartDashboard.putNumber("Controller_FlywheelL_BusVoltage", mFlywheelL.getBusVoltage());
        SmartDashboard.putNumber("Controller_FlywheelL_AppliedOutput", mFlywheelL.getAppliedOutput());
        SmartDashboard.putNumber("Controller_FlywheelL_DeviceId", mFlywheelL.getDeviceId());
        SmartDashboard.putNumber("Controller_FlywheelL_Faults", mFlywheelL.getFaults());
        SmartDashboard.putNumber("Controller_FlywheelL_Get", mFlywheelL.get());
        SmartDashboard.putString("Controller_FlywheelL_FirmwareString", mFlywheelL.getFirmwareString());
        SmartDashboard.putString("Controller_FlywheelL_IdleMode", mFlywheelL.getIdleMode().toString());
        SmartDashboard.putString("Controller_FlywheelL_SerialNumber", mFlywheelL.getSerialNumber().toString());
        SmartDashboard.putNumber("Controller_FlywheelL_StickyFaults", mFlywheelL.getStickyFaults());
        SmartDashboard.putNumber("Controller_FlywheelL_VoltageCompensationNominalVoltage", mFlywheelL.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("Controller_FlywheelL_MotorType", mFlywheelL.getMotorType().toString());
      }
          
        
   }
    

    

