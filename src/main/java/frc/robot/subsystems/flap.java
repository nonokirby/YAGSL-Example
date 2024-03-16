package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.*;

import java.lang.Math;

public class flap extends SubsystemBase{
  CANSparkMax mflap = new CANSparkMax(ampFlap.id, ampFlap.neo);
  double speed;
  DigitalInput sw = new DigitalInput(limitSwitch.dioId);
    public void set(double input_speed){
      double speedL = input_speed * climberL.power;
      double speedR = input_speed * climberR.power;
      mflap.set(speedL);
      mflap.set(speedR);
    }
    public void out(int position){
        if (position == 0 || !sw.get())
         mflap.set(1);
      } 
}
