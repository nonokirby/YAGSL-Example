package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.shooter;

import java.lang.Math;

public class flap extends SubsystemBase{
  CANSparkMax mflap = new CANSparkMax(ampFlap.id, ampFlap.neo);
  double speed;
  DigitalInput sw = new DigitalInput(limitSwitch.dioId); 
  PIDController pid = new PIDController(ampFlap.Kp, ampFlap.Ki, ampFlap.Kd);
    public void set(double input_speed){
      double speedL = input_speed * climberL.power;
      double speedR = input_speed * climberR.power;
      mflap.set(speedL);
      mflap.set(speedR);
    }

    public void setpoint(double setpoint){
      pid.calculate(mflap.getEncoder().getPosition(), setpoint);
    }
    
    public void zero(){
      mflap.set(-1);
      if (sw.get()){
        mflap.set(0);
        mflap.getEncoder().setPosition(0);
      }
    }
    public void out(int position){
        if (position == 0 || !sw.get()){
         mflap.set(1);}
  } 
}
