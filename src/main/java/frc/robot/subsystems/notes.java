package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;






public class notes extends SubsystemBase {
  
  CANSparkMax mIntake = new CANSparkMax(Constants.id_sht_intake, Constants.k_sht_intake);
  CANSparkMax mFlywheelL = new CANSparkMax(Constants.id_sht_flywheel_l, Constants.k_sht_flywheel_l);
  CANSparkMax mFlywheelR = new CANSparkMax(Constants.id_sht_flywheel_r, Constants.k_sht_flywheel_r);
  CANSparkMax mFeederL = new CANSparkMax(Constants.id_sht_feeder_l, Constants.k_sht_feeder_l);
  CANSparkMax mFeederR = new CANSparkMax(Constants.id_sht_feeder_r, Constants.k_sht_feeder_r);

    public void intake(double speed){
      mIntake.set(speed);
    }
    public void flywheel(double speed){
      mFlywheelL.set(speed);
      mFlywheelR.set(speed * -1);
    }
    //feed
    public void feed(double speed){
      mFeederL.set(speed);
      mFeederR.set(speed * -1);
    }
  
}
