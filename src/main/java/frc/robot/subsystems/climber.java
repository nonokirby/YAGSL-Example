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
        SmartDashboard.putNumber("/Climber/LeftEncoder/Position", encoderL.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("/Climber/LeftEncoder/Velocity", encoderL.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("/Climber/LeftEncoder/DeviceID", encoderL.getDeviceID());
        SmartDashboard.putNumber("/Climber/LeftEncoder/SupplyVoltage", encoderL.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putString("/Climber/LeftEncoder/Version", encoderL.getVersion().toString());
        SmartDashboard.putString("/Climber/LeftEncoder/FaultField", encoderL.getFaultField().toString());

        SmartDashboard.putNumber("/Climber/ClimberL/Position", mClimberL.getEncoder().getPosition());
        SmartDashboard.putNumber("/Climber/ClimberL/Velocity", mClimberL.getEncoder().getVelocity());
        SmartDashboard.putNumber("/Climber/ClimberL/Temperature", mClimberL.getMotorTemperature());
        SmartDashboard.putNumber("/Climber/ClimberL/Current", mClimberL.getOutputCurrent());
        SmartDashboard.putNumber("/Climber/ClimberL/BusVoltage", mClimberL.getBusVoltage());
        SmartDashboard.putNumber("/Climber/ClimberL/AppliedOutput", mClimberL.getAppliedOutput());
        SmartDashboard.putNumber("/Climber/ClimberL/Id", mClimberL.getDeviceId());
        SmartDashboard.putNumber("/Climber/ClimberL/Faults", mClimberL.getFaults());
        SmartDashboard.putString("/Climber/ClimberL/Mode", mClimberL.getIdleMode().toString());
        SmartDashboard.putString("/Climber/ClimberL/FirmwareString", mClimberL.getFirmwareString());
        SmartDashboard.putBoolean("/Climber/ClimberL/Inverted", mClimberL.getInverted());
        SmartDashboard.putString("/Climber/ClimberL/SerialNumber", mClimberL.getSerialNumber().toString());
        SmartDashboard.putNumber("/Climber/ClimberL/VoltageCompensationNominalVoltage", mClimberL.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("/Climber/ClimberL/MotorType", mClimberL.getMotorType().toString());
        SmartDashboard.putNumber("/Climber/ClimberL/MotorOutput", mClimberL.get());

        SmartDashboard.putNumber("/Climber/ClimberR/Position", mClimberR.getEncoder().getPosition());
        SmartDashboard.putNumber("/Climber/ClimberR/Velocity", mClimberR.getEncoder().getVelocity());
        SmartDashboard.putNumber("/Climber/ClimberR/Temperature", mClimberR.getMotorTemperature());
        SmartDashboard.putNumber("/Climber/ClimberR/Current", mClimberR.getOutputCurrent());
        SmartDashboard.putNumber("/Climber/ClimberR/BusVoltage", mClimberR.getBusVoltage());
        SmartDashboard.putNumber("/Climber/ClimberR/AppliedOutput", mClimberR.getAppliedOutput());
        SmartDashboard.putNumber("/Climber/ClimberR/Id", mClimberR.getDeviceId());
        SmartDashboard.putNumber("/Climber/ClimberR/Faults", mClimberR.getFaults());
        SmartDashboard.putString("/Climber/ClimberR/Mode", mClimberR.getIdleMode().toString());
        SmartDashboard.putString("/Climber/ClimberR/FirmwareString", mClimberR.getFirmwareString());
        SmartDashboard.putBoolean("/Climber/ClimberR/Inverted", mClimberR.getInverted());
        SmartDashboard.putString("/Climber/ClimberR/SerialNumber", mClimberR.getSerialNumber().toString());
        SmartDashboard.putNumber("/Climber/ClimberR/VoltageCompensationNominalVoltage", mClimberR.getVoltageCompensationNominalVoltage());
        SmartDashboard.putString("/Climber/ClimberR/MotorType", mClimberR.getMotorType().toString());
        SmartDashboard.putNumber("/Climber/ClimberR/MotorOutput", mClimberR.get());
        

        
        
        
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
