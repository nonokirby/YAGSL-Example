package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climberL;
import frc.robot.Constants.climberLim;
import frc.robot.Constants.climberR;

import java.lang.Math;
import com.revrobotics.CANSparkMax.*;

import javax.swing.Renderer;

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
  
        if (SmartDashboard.getNumber("LeftEncoder", -1442343243) == -1442343243){
          SmartDashboard.putNumber("LeftEncoder", encoderL.getPosition().getValueAsDouble());
        }
        else{
          encoderL.setPosition(SmartDashboard.getNumber("LeftEncoder", 0));
        }
        if (SmartDashboard.getNumber("RightEncoder", -1442343243) == -1442343243){
          SmartDashboard.putNumber("RightEncoder", encoderR.getPosition().getValueAsDouble());
        }
        else{
          encoderR.setPosition(SmartDashboard.getNumber("RightEncoder", 0));
        }
  
        
      }   
      else{
        SmartDashboard.putNumber("LeftEncoder", encoderL.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("RightEncoder", encoderR.getPosition().getValueAsDouble());
        SmartDashboard.setPersistent("LeftEncoder");
        SmartDashboard.setPersistent("RightEncoder");
      }
     }
    public void set(double input_speed){
      double speedL = input_speed * climberL.power;
      double speedR = input_speed * climberR.power;
      mClimberL.set(speedL);
      mClimberR.set(speedR);
      //pid.calculate(getPosition(), position)

    }
    public void goSet(int setpoint){
      mClimberL.set(kPID.calculate(encoderL.getPosition().getValueAsDouble(), setpoint));
      mClimberR.set(kPID.calculate(encoderR.getPosition().getValueAsDouble(), setpoint));
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
