// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static class intake{
    public static CANSparkLowLevel.MotorType neo = MotorType.kBrushless;
    public static int id  = 13;
    public static double power  = 1; // Output speed multiplier  -1 to 1
    public static int current = 30;
    public static IdleMode idle = IdleMode.kCoast;
  }
  public static class flywheelL{
    public static CANSparkLowLevel.MotorType neo = MotorType.kBrushless;
    public static int id  = 14;
    public static double power  = 1; // Output speed multiplier  -1 to 1
    public static int current = 40;
    public static IdleMode idle = IdleMode.kCoast;
  }
  public static class flywheelR{
    public static CANSparkLowLevel.MotorType neo = MotorType.kBrushless;
    public static int id  = 15;
    public static double power  = -1; // Output speed multiplier  -1 to 1
    public static int current = 40;
    public static IdleMode idle = IdleMode.kCoast;
  }
  public static class feederL{
    public static CANSparkLowLevel.MotorType neo = MotorType.kBrushless;
    public static int id  = 16;
    public static double power  = 1; // Output speed multiplier  -1 to 1
    public static int current = 30;
    public static IdleMode idle = IdleMode.kBrake;
  }
  public static class feederR{
    public static CANSparkLowLevel.MotorType neo = MotorType.kBrushless;
    public static int id  = 17;
    public static double power  = -1; // Output speed multiplier  -1 to 1
    public static int current = 30;
    public static IdleMode idle = IdleMode.kBrake;
  }
  public static class climberL{
    public static CANSparkLowLevel.MotorType neo = MotorType.kBrushless;
    public static int id  = 18;
    public static double power  = 1; // Output speed multiplier  -1 to 1
    public static int current = 40;
    public static IdleMode idle = IdleMode.kBrake;
    public static int encoderid = 22;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double ConversionRate = 1024; //Div encoder value by this so 0 = bottom of climber & 1 = top
  }
  public static class climberR{
    public static CANSparkLowLevel.MotorType neo = MotorType.kBrushless;
    public static int id  = 19;
    public static double power  = 1; // Output speed multiplier  -1 to 1
    public static int current = 40;
    public static IdleMode idle = IdleMode.kBrake;
    public static int encoderid = 21;
  }
  public static class ampFlap{
    public static CANSparkLowLevel.MotorType neo = MotorType.kBrushless;
    public static CANSparkLowLevel.MotorType old = MotorType.kBrushed;
    public static int id  = 20;
    public static double power  = 1; // Output speed multiplier  -1 to 1 
    public static int current = 30;
    public static IdleMode idle = IdleMode.kBrake;
  }

  public static class limitSwitch{
    public static int dioId  = 0; //Port No. on DIO
    public static double setpointTime = 1; //Time in seconds to move the motor in the opposite direction for the flap
  }


  public static class climberLim{
    public static int max = 2048; // in encoder ticks. 1024 tks = 360 deg
    public static int mid = 0; // in encoder ticks. 1024 tks = 360 deg
    public static int deviation = 250; // in encoder ticks. 1024 tks = 360 deg  defines how far from max or min the encoder can be
    public static double curveDiv = 2;  
    public static double curveMult = .1;
  }

  public static class climberPID{
  }

  
  

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants{
    public static final int XBOX_DRIVER_PORT = 0;
    public static final int XBOX_SHOOTER_PORT = 1;
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }  
  public static class Climber{
    public static final double GetKp = 0;
    public static final double GetKi = 0;
    public static final double GetKd = 0;
  }

  public static class Flap{
    public static final double GetKp = 0;
    public static final double GetKi = 0;
    public static final double GetKd = 0;
  }
}
