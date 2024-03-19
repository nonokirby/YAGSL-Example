package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climberL;
import frc.robot.Constants.climberR;

public class climber extends SubsystemBase{

  CANSparkMax mClimberL = new CANSparkMax(climberL.id, climberL.neo);
  CANSparkMax mClimberR = new CANSparkMax(climberR.id, climberR.neo);
  CANcoder encoderL = new CANcoder(climberL.encoderid);
  CANcoder encoderR = new CANcoder(climberR.encoderid);
  //  leftencoderpos = encoderL.getPosition().getValueAsDouble();
  


  PIDController kPID = new PIDController(climberL.Kp,climberL.Ki,climberL.Kd);
  int rEncoderDistance;
  double speedL;
  double speedR;
    public void periodic(){
       if (SmartDashboard.getBoolean("First Setup", true)){
        SmartDashboard.putBoolean("First Setup", false);
  
        mClimberL.clearFaults();
        mClimberL.setIdleMode(IdleMode.kBrake);
        mClimberL.setSmartCurrentLimit(climberL.current);
        mClimberR.clearFaults();
        mClimberR.setIdleMode(IdleMode.kBrake);
        mClimberR.setSmartCurrentLimit(climberR.current);
  
      //   if (SmartDashboard.getNumber("LeftEncoder", -1442343243) == -1442343243){
      //     SmartDashboard.putNumber("LeftEncoder", encoderL.getPosition().getValueAsDouble());
      //   }
      //   else{
      //     encoderL.setPosition(SmartDashboard.getNumber("LeftEncoder", 0));
      //   }
      //   if (SmartDashboard.getNumber("RightEncoder", -1442343243) == -1442343243){
      //     SmartDashboard.putNumber("RightEncoder", encoderR.getPosition().getValueAsDouble());
      //   }
      //   else{
      //     encoderR.setPosition(SmartDashboard.getNumber("RightEncoder", 0));
      //   }
  
        
      }   
      // // else{
        SmartDashboard.putNumber("LeftEncoder", encoderL.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("RightEncoder", encoderR.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("LeftEncoderVelocity", encoderL.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("RightEncoderVelocity", encoderR.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("LeftEncoderDeviceID", encoderL.getDeviceID());
        SmartDashboard.putNumber("RightEncoderDeviceID", encoderR.getDeviceID());
        SmartDashboard.putNumber("LeftEncoderSupplyVoltage", encoderL.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("RightEncoderSupplyVoltage", encoderR.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putString("LeftEncoderVersion", encoderL.getVersion().toString());
        SmartDashboard.putString("RightEncoderVersion", encoderR.getVersion().toString());
        SmartDashboard.putString("LeftEncoderFaultField", encoderL.getFaultField().toString());
        SmartDashboard.putString("RightEncoderFaultField", encoderR.getFaultField().toString());


        SmartDashboard.putNumber("NeoClimberLPos", mClimberL.getEncoder().getPosition());
        SmartDashboard.putNumber("NeoClimberLVel", mClimberL.getEncoder().getVelocity());
        SmartDashboard.putNumber("NeoClimberRPos", mClimberR.getEncoder().getPosition());
        SmartDashboard.putNumber("NeoClimberRVel", mClimberR.getEncoder().getVelocity());
        SmartDashboard.putNumber("NeoClimberLTemp", mClimberL.getMotorTemperature());
        SmartDashboard.putNumber("NeoClimberRTemp", mClimberR.getMotorTemperature());

        SmartDashboard.putNumber("ControllerClimberLCurrent", mClimberL.getOutputCurrent());
        SmartDashboard.putNumber("ControllerClimberRCurrent", mClimberR.getOutputCurrent());
        SmartDashboard.putNumber("ControllerClimberLBusVoltage", mClimberL.getBusVoltage());
        SmartDashboard.putNumber("ControllerClimberRBusVoltage", mClimberR.getBusVoltage());
        SmartDashboard.putNumber("ControllerClimberLAppliedOutput", mClimberL.getAppliedOutput());
        SmartDashboard.putNumber("ControllerClimberRAppliedOutput", mClimberR.getAppliedOutput());
        SmartDashboard.putNumber("ControllerClimberLId", mClimberL.getDeviceId());
        SmartDashboard.putNumber("ControllerClimberRId", mClimberR.getDeviceId());
        SmartDashboard.putNumber("ControllerClimberLBusVoltage", mClimberL.getFaults());
        SmartDashboard.putNumber("ControllerClimberRBusVoltage", mClimberR.getFaults());
        SmartDashboard.putString("ControllerClimberLMode", mClimberL.getIdleMode().toString());
        SmartDashboard.putString("ControllerClimberRMode", mClimberR.getIdleMode().toString());
        
        SmartDashboard.putString("ControllerClimberLFirmwareString", mClimberL.getFirmwareString());
        SmartDashboard.putBoolean("ControllerClimberLInverted", mClimberL.getInverted());
        SmartDashboard.putString("ControllerClimberLSerialNumber", mClimberL.getSerialNumber().toString());
        SmartDashboard.putNumber("ControllerClimberLVoltageCompensationNominalVoltage", mClimberL.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("ControllerClimberLMotorType", mClimberL.getMotorType().toString());
        SmartDashboard.putNumber("ControllerClimberLMotorOutput", mClimberL.get());

        SmartDashboard.putString("ControllerClimberRFirmwareString", mClimberR.getFirmwareString());
        SmartDashboard.putBoolean("ControllerClimberRInverted", mClimberR.getInverted());
        SmartDashboard.putString("ControllerClimberRSerialNumber", mClimberR.getSerialNumber().toString());
        SmartDashboard.putNumber("ControllerClimberRVoltageCompensationNominalVoltage", mClimberR.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("ControllerClimberRMotorType", mClimberR.getMotorType().toString());
        SmartDashboard.putNumber("ControllerClimberRMotorOutput", mClimberR.get());
        

        
        
        
        // SmartDashboard.putNumber()
        // SmartDashboard.setPersistent("LeftEncoder");
        // SmartDashboard.setPersistent("RightEncoder");
      // }
     }
    public void set(double input_speed){
      double speedL = input_speed * climberL.power;
      double speedR = input_speed * climberR.power;
      mClimberL.set(speedL);
      mClimberR.set(speedR);
      //pid.calculate(getPosition(), position)

    }
    public void goSet(double setpoint){
      //sets modEncoderX to be a number where 0 is the bottom of the climber's state and 1 is the top
      double modEncoderL = encoderL.getPosition().getValueAsDouble()/climberL.ConversionRate; 
      double modencoderR = encoderR.getPosition().getValueAsDouble()/climberL.ConversionRate;
      mClimberL.set(kPID.calculate(modEncoderL, setpoint));
      mClimberR.set(kPID.calculate(modencoderR, setpoint));
      //mClimberR.set(pid.calculate(rEncoderDistance,setpoint));
      
    }

    
    //basically ram the climber into itself until it stops repeatedly to determine zero point for cancoders
    //somewhat like a prusa 3d printer
    public void zero(){
      int attempts = 3; 
      while (!(attempts==0)) {
        mClimberL.set(-1);
        mClimberR.set(-1);
        boolean LeftZero = false;
        boolean RightZero = false;
      while (!LeftZero || !RightZero){
      if (mClimberL.getEncoder().getVelocity() < 10){ 
        encoderL.setPosition(0);
        mClimberL.set(0);
        LeftZero = true;
      }
      if (mClimberR.getEncoder().getVelocity() < 10){ 
        encoderR.setPosition(0);
        mClimberR.set(0);
        RightZero = true;
      }
     }
     attempts--;
     mClimberL.set(1);
     mClimberR.set(1);
     while (mClimberL.getEncoder().getVelocity() < 300); 
    }
    encoderL.setPosition(0);
    encoderR.setPosition(0);
    }

    public void setInd(double input_speedL, double input_speedR){
      double speedL = input_speedL * climberL.power;
      double speedR = input_speedR * climberR.power;
      mClimberL.set(speedL);
      mClimberR.set(speedR);
    }
}
