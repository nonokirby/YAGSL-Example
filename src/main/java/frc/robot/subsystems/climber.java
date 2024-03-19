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

  public CANSparkMax mClimberL = new CANSparkMax(climberL.id, climberL.neo);
  public CANSparkMax mClimberR = new CANSparkMax(climberR.id, climberR.neo);
  public CANcoder encoderL = new CANcoder(climberL.encoderid);
  public CANcoder encoderR = new CANcoder(climberR.encoderid);
  //  leftencoderpos = encoderL.getPosition().getValueAsDouble();
  


  PIDController kPID = new PIDController(climberL.Kp,climberL.Ki,climberL.Kd);
  int rEncoderDistance;
  double speedL;
  double speedR;
    public void setup(){
        mClimberL.clearFaults();
        mClimberL.setIdleMode(IdleMode.kBrake);
        mClimberL.setSmartCurrentLimit(climberL.current);
        mClimberR.clearFaults();
        mClimberR.setIdleMode(IdleMode.kBrake);
        mClimberR.setSmartCurrentLimit(climberR.current);
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
     while (mClimberL.getEncoder().getVelocity() < 1500); 
    }
  }

    public void setInd(double input_speedL, double input_speedR){
      double speedL = input_speedL * climberL.power;
      double speedR = input_speedR * climberR.power;
      mClimberL.set(speedL);
      mClimberR.set(speedR);
    }
}
