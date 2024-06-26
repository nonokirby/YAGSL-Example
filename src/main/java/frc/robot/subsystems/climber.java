package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climberL;
import frc.robot.Constants.climberR;

public class climber extends SubsystemBase{

  public CANSparkMax mClimberL = new CANSparkMax(climberL.id, climberL.neo);
  public CANSparkMax mClimberR = new CANSparkMax(climberR.id, climberR.neo);
  public CANcoder encoderL = new CANcoder(climberL.encoderid);
  public CANcoder encoderR = new CANcoder(climberR.encoderid);
  PIDController kPID = new PIDController(climberL.Kp, climberL.Ki, climberL.Kd);

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
    }

    public void goSet(double setpoint){
      //sets modEncoderX to be a number where 0 is the bottom of the climber's state and 1 is the top
      double modEncoderL = encoderL.getPosition().getValueAsDouble()/climberL.ConversionRate; 
      double modencoderR = encoderR.getPosition().getValueAsDouble()/climberL.ConversionRate;
      mClimberL.set(kPID.calculate(encoderL.getPosition().getValueAsDouble(), setpoint));
      mClimberR.set(kPID.calculate(encoderL.getPosition().getValueAsDouble(), setpoint));
      SmartDashboard.putNumber("Climber/ClimberL/Setpoint", setpoint);
      SmartDashboard.putNumber("Climber/ClimberL/Setpoint", setpoint);
    }

    
    //basically ram the climber into itself until it stops repeatedly to determine zero point for cancoders
    //somewhat like a prusa 3d printer
    public void zero(){
      int attempts = 2; 
      mClimberL.set(-1);
      mClimberR.set(-1);
      while (!(attempts==0)) {

        boolean LeftZero = false;
        boolean RightZero = false;

      while (!LeftZero || !RightZero){


      if (mClimberL.getEncoder().getVelocity() < 300){ 
        encoderL.setPosition(0);
        mClimberL.set(0);
        LeftZero = true;
      }
      else{
        mClimberL.set(-1);
      }

      if (mClimberR.getEncoder().getVelocity() < 300){ 
        encoderR.setPosition(0);
        mClimberR.set(0);
        RightZero = true;
      }
      else{
        mClimberR.set(-1);
      }
     }

     attempts--;
      mClimberL.set(kPID.calculate(encoderL.getPosition().getValueAsDouble(), 1000));
      mClimberR.set(kPID.calculate(encoderR.getPosition().getValueAsDouble(), 1000));
     
    }
  }
    public void setInd(double input_speedL, double input_speedR){
      double speedL = input_speedL * climberL.power;
      double speedR = input_speedR * climberR.power;
      mClimberL.set(speedL);
      mClimberR.set(speedR);
    }
}
