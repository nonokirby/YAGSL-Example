package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.climberL;
import frc.robot.subsystems.Constants.climberLim;
import frc.robot.subsystems.Constants.climberR;

import java.lang.Math;
import com.revrobotics.CANSparkMax.*;

import javax.swing.Renderer;

public class climber extends SubsystemBase{

  double Kp;
  double Ki;
  double Kd;
  CANSparkMax mClimberL = new CANSparkMax(climberL.id, climberL.neo);
  CANSparkMax mClimberR = new CANSparkMax(climberR.id, climberR.neo);


  PIDController pid = new PIDController(Kp, Ki, Kd);
  int rEncoderDistance;
  double speedL;
  double speedR;
    
    public void setup(){
      mClimberL.clearFaults();
      mClimberL.setIdleMode(IdleMode.kBrake);
      mClimberL.setSmartCurrentLimit(climberL.current);
    }
    public void set(double input_speed){
      double speedL = input_speed * climberL.power;
      double speedR = input_speed * climberR.power;
      mClimberL.set(speedL);
      mClimberR.set(speedR);
      
    }
    public void go(int setpoint){
      //mClimberL.set(pid.calculate(mClimberL.SparkAbsoluteEncoder.getPosition(),setpoint));
      //mClimberR.set(pid.calculate(rEncoderDistance,setpoint));
    }
    public void zero(){
      
    }


    /*public void climb(int position){
      int current = 1;
      int diff = current - position;
      if (Math.abs(diff) > climberLim.deviation ){
        mClimberL.set((((
            Math.abs(diff)/1024 )
          / climberLim.curveDiv)
          * climberLim.curveMult)
          * climberL.power);
         mClimberR.set((((
            Math.abs(diff)/1024 )
          / climberLim.curveDiv)
          * climberLim.curveMult)
          * climberR.power);
      }
    }*/

    
    public void setInd(double input_speedL, double input_speedR){
      double speedL = input_speedL * climberL.power;
      double speedR = input_speedR * climberR.power;
      mClimberL.set(speedL);
      mClimberR.set(speedR);
    }
}
