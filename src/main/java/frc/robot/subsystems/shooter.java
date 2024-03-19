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
    public void setup(){

    }

   /* public void periodic(){
        double flywheelSpeedL = RobotContainer.shooterXbox.getLeftTriggerAxis() * flywheelL.power;
        double flywheelSpeedR = RobotContainer.shooterXbox.getLeftTriggerAxis() * flywheelR.power;
        mFlywheelL.set(flywheelSpeedL);
        mFlywheelR.set(flywheelSpeedR);
      }*/
      public void periodic(){
        // FlywheelL
        SmartDashboard.putNumber("/Shooter/FeederL/Position", mFeederL.getEncoder().getPosition());
        SmartDashboard.putNumber("/Shooter/FeederL/Velocity", mFeederL.getEncoder().getVelocity());
        SmartDashboard.putNumber("/Shooter/FeederL/Current", mFeederL.getOutputCurrent());
        SmartDashboard.putNumber("/Shooter/FeederL/Temperature", mFeederL.getMotorTemperature());
        SmartDashboard.putNumber("/Shooter/FeederL/BusVoltage", mFeederL.getBusVoltage());
        SmartDashboard.putNumber("/Shooter/FeederL/AppliedOutput", mFeederL.getAppliedOutput());
        SmartDashboard.putNumber("/Shooter/FeederL/DeviceId", mFeederL.getDeviceId());
        SmartDashboard.putNumber("/Shooter/FeederL/Faults", mFeederL.getFaults());
        SmartDashboard.putNumber("/Shooter/FeederL/Get", mFeederL.get());
        SmartDashboard.putString("/Shooter/FeederL/FirmwareString", mFeederL.getFirmwareString());
        SmartDashboard.putString("/Shooter/FeederL/IdleMode", mFeederL.getIdleMode().toString());
        SmartDashboard.putString("/Shooter/FeederL/SerialNumber", mFeederL.getSerialNumber().toString());
        SmartDashboard.putNumber("/Shooter/FeederL/StickyFaults", mFeederL.getStickyFaults());
        SmartDashboard.putNumber("/Shooter/FeederL/VoltageCompensationNominalVoltage", mFeederL.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("/Shooter/FeederL/MotorType", mFeederL.getMotorType().toString());

        // FeederR
        SmartDashboard.putNumber("/Shooter/FeederR/Position", mFeederR.getEncoder().getPosition());
        SmartDashboard.putNumber("/Shooter/FeederR/Velocity", mFeederR.getEncoder().getVelocity());
        SmartDashboard.putNumber("/Shooter/FeederR/Current", mFeederR.getOutputCurrent());
        SmartDashboard.putNumber("/Shooter/FeederR/Temperature", mFeederR.getMotorTemperature());
        SmartDashboard.putNumber("/Shooter/FeederR/BusVoltage", mFeederR.getBusVoltage());
        SmartDashboard.putNumber("/Shooter/FeederR/AppliedOutput", mFeederR.getAppliedOutput());
        SmartDashboard.putNumber("/Shooter/FeederR/DeviceId", mFeederR.getDeviceId());
        SmartDashboard.putNumber("/Shooter/FeederR/Faults", mFeederR.getFaults());
        SmartDashboard.putNumber("/Shooter/FeederR/Get", mFeederR.get());
        SmartDashboard.putString("/Shooter/FeederR/FirmwareString", mFeederR.getFirmwareString());
        SmartDashboard.putString("/Shooter/FeederR/IdleMode", mFeederR.getIdleMode().toString());
        SmartDashboard.putString("/Shooter/FeederR/SerialNumber", mFeederR.getSerialNumber().toString());
        SmartDashboard.putNumber("/Shooter/FeederR/StickyFaults", mFeederR.getStickyFaults());
        SmartDashboard.putNumber("/Shooter/FeederR/VoltageCompensationNominalVoltage", mFeederR.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("/Shooter/FeederR/MotorType", mFeederR.getMotorType().toString());

        // Intake
        SmartDashboard.putNumber("/Shooter/Intake/Position", mIntake.getEncoder().getPosition());
        SmartDashboard.putNumber("/Shooter/Intake/Velocity", mIntake.getEncoder().getVelocity());
        SmartDashboard.putNumber("/Shooter/Intake/Current", mIntake.getOutputCurrent());
        SmartDashboard.putNumber("/Shooter/Intake/Temperature", mIntake.getMotorTemperature());
        SmartDashboard.putNumber("/Shooter/Intake/BusVoltage", mIntake.getBusVoltage());
        SmartDashboard.putNumber("/Shooter/Intake/AppliedOutput", mIntake.getAppliedOutput());
        SmartDashboard.putNumber("/Shooter/Intake/DeviceId", mIntake.getDeviceId());
        SmartDashboard.putNumber("/Shooter/Intake/Faults", mIntake.getFaults());
        SmartDashboard.putNumber("/Shooter/Intake/Get", mIntake.get());
        SmartDashboard.putString("/Shooter/Intake/FirmwareString", mIntake.getFirmwareString());
        SmartDashboard.putString("/Shooter/Intake/IdleMode", mIntake.getIdleMode().toString());
        SmartDashboard.putString("/Shooter/Intake/SerialNumber", mIntake.getSerialNumber().toString());
        SmartDashboard.putNumber("/Shooter/Intake/StickyFaults", mIntake.getStickyFaults());
        SmartDashboard.putNumber("/Shooter/Intake/VoltageCompensationNominalVoltage", mIntake.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("/Shooter/Intake/MotorType", mIntake.getMotorType().toString());

        // FlywheelR
        SmartDashboard.putNumber("/Shooter/FlywheelR/Position", mFlywheelR.getEncoder().getPosition());
        SmartDashboard.putNumber("/Shooter/FlywheelR/Velocity", mFlywheelR.getEncoder().getVelocity());
        SmartDashboard.putNumber("/Shooter/FlywheelR/Current", mFlywheelR.getOutputCurrent());
        SmartDashboard.putNumber("/Shooter/FlywheelR/Temperature", mFlywheelR.getMotorTemperature());
        SmartDashboard.putNumber("/Shooter/FlywheelR/BusVoltage", mFlywheelR.getBusVoltage());
        SmartDashboard.putNumber("/Shooter/FlywheelR/AppliedOutput", mFlywheelR.getAppliedOutput());
        SmartDashboard.putNumber("/Shooter/FlywheelR/DeviceId", mFlywheelR.getDeviceId());
        SmartDashboard.putNumber("/Shooter/FlywheelR/Faults", mFlywheelR.getFaults());
        SmartDashboard.putNumber("/Shooter/FlywheelR/Get", mFlywheelR.get());
        SmartDashboard.putString("/Shooter/FlywheelR/FirmwareString", mFlywheelR.getFirmwareString());
        SmartDashboard.putString("/Shooter/FlywheelR/IdleMode", mFlywheelR.getIdleMode().toString());
        SmartDashboard.putString("/Shooter/FlywheelR/SerialNumber", mFlywheelR.getSerialNumber().toString());
        SmartDashboard.putNumber("/Shooter/FlywheelR/StickyFaults", mFlywheelR.getStickyFaults());
        SmartDashboard.putNumber("/Shooter/FlywheelR/VoltageCompensationNominalVoltage", mFlywheelR.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("/Shooter/FlywheelR/MotorType", mFlywheelR.getMotorType().toString());

        // FlywheelL
        SmartDashboard.putNumber("/Shooter/FlywheelL/Position", mFlywheelL.getEncoder().getPosition());
        SmartDashboard.putNumber("/Shooter/FlywheelL/Velocity", mFlywheelL.getEncoder().getVelocity());
        SmartDashboard.putNumber("/Shooter/FlywheelL/Current", mFlywheelL.getOutputCurrent());
        SmartDashboard.putNumber("/Shooter/FlywheelL/Temperature", mFlywheelL.getMotorTemperature());
        SmartDashboard.putNumber("/Shooter/FlywheelL/BusVoltage", mFlywheelL.getBusVoltage());
        SmartDashboard.putNumber("/Shooter/FlywheelL/AppliedOutput", mFlywheelL.getAppliedOutput());
        SmartDashboard.putNumber("/Shooter/FlywheelL/DeviceId", mFlywheelL.getDeviceId());
        SmartDashboard.putNumber("/Shooter/FlywheelL/Faults", mFlywheelL.getFaults());
        SmartDashboard.putNumber("/Shooter/FlywheelL/Get", mFlywheelL.get());
        SmartDashboard.putString("/Shooter/FlywheelL/FirmwareString", mFlywheelL.getFirmwareString());
        SmartDashboard.putString("/Shooter/FlywheelL/IdleMode", mFlywheelL.getIdleMode().toString());
        SmartDashboard.putString("/Shooter/FlywheelL/SerialNumber", mFlywheelL.getSerialNumber().toString());
        SmartDashboard.putNumber("/Shooter/FlywheelL/StickyFaults", mFlywheelL.getStickyFaults());
        SmartDashboard.putNumber("/Shooter/FlywheelL/VoltageCompensationNominalVoltage", mFlywheelL.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("/Shooter/FlywheelL/MotorType", mFlywheelL.getMotorType().toString());
      }
          
        
   }
    

    

