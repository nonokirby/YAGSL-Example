// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.climber.down;
import frc.robot.commands.climber.left;
import frc.robot.commands.climber.right;
import frc.robot.commands.climber.up;
import frc.robot.commands.flap.lower;
import frc.robot.commands.flap.raise;
import frc.robot.commands.shooter.eject;
import frc.robot.commands.shooter.feed;
import frc.robot.commands.shooter.flywheel;
import frc.robot.commands.shooter.intake;
import frc.robot.commands.shooter.spit;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Logging;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  public final static frc.robot.subsystems.climber climber = new frc.robot.subsystems.climber();
  public final static frc.robot.subsystems.shooter shooter = new frc.robot.subsystems.shooter();
  public final static frc.robot.subsystems.Logging Logging = new frc.robot.subsystems.Logging();
  public final static frc.robot.subsystems.flap flap = new frc.robot.subsystems.flap();
   
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
                                                                    
  public SendableChooser<String> autochooser  = new SendableChooser<String>();
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  public XboxController driverXbox = new XboxController(OperatorConstants.XBOX_DRIVER_PORT);
  CommandXboxController shooterXbox = new CommandXboxController(OperatorConstants.XBOX_SHOOTER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    @SuppressWarnings("unused") 
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox::getYButtonPressed,
                                                                   driverXbox::getAButtonPressed,
                                                                   driverXbox::getXButtonPressed,
                                                                   driverXbox::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());
    

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    @SuppressWarnings("unused")
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband((driverXbox.getLeftY()) * -1,OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband((driverXbox.getLeftX()) * -1, OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));


    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));    

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    
    NamedCommands.registerCommand("Shoot", new flywheel().withTimeout(.5).andThen(new feed().withTimeout(1)));
    NamedCommands.registerCommand("Intake", new intake().withTimeout(3));
    NamedCommands.registerCommand("Lower Note", new spit().withTimeout(.1));

    autochooser.setDefaultOption("Do nothing", new String("Nothing"));
    autochooser.addOption("Shoot", new String("Shoot Taxi Intake"));
    autochooser.addOption("Taxi", new String("Taxi Only"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    

    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    new JoystickButton(driverXbox, 2).whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    /*shooterXbox.rightTrigger().whileTrue(new spit());
    shooterXbox.rightBumper().whileTrue(new feed());
    shooterXbox.a().whileTrue(new raise());
    shooterXbox.b().whileTrue(new lower());
    shooterXbox.leftBumper().whileTrue(new intake());
    shooterXbox.leftTrigger().whileTrue(new flywheel());*/
    shooterXbox.start().whileTrue(new eject());
    shooterXbox.leftBumper().whileTrue(new intake()).onFalse(new spit().withTimeout(.1));
    shooterXbox.rightBumper().whileTrue(new flywheel().withTimeout(.5).andThen(new feed()));
    new JoystickButton(shooterXbox.getHID(), 1).whileTrue(new left());
  
    







    //Dpad for Climber

    new POVButton(shooterXbox.getHID(),  45).whileTrue(new up());
    new POVButton(shooterXbox.getHID(),   0).whileTrue(new up());
    new POVButton(shooterXbox.getHID(), 315).whileTrue(new up());
    new POVButton(shooterXbox.getHID(), 135).whileTrue(new down());
    new POVButton(shooterXbox.getHID(), 225).whileTrue(new down());
    new POVButton(shooterXbox.getHID(), 180).whileTrue(new down());
    new POVButton(shooterXbox.getHID(), 90).whileTrue(new right());
    new POVButton(shooterXbox.getHID(), 270).whileTrue(new left());

    
  //    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *6
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand(autochooser.getSelected());
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
} 
// ltrigger rev flywheels done
// rtrigger run feeder done
// climber up and down on pov
// climber tilt on pov 
// left brings up left side, right brings up right side
// left bumper triggers intake 
// a puts flap out
// b brings flap in


//driver 
// 
