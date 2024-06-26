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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.commands.shooter.flywheelSpit;
import frc.robot.commands.shooter.intake;
import frc.robot.commands.shooter.spit;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Logging;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
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
  private final SendableChooser<Command> autoChooser;
                        
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
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightY(), OperatorConstants.RIGHT_X_DEADBAND));
    

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
    
    NamedCommands.registerCommand("Shoot", new flywheel().withTimeout(4).andThen(new feed().withTimeout(.3)).andThen(new flywheelSpit().withTimeout(.5)));
    NamedCommands.registerCommand("Intake", new intake().withTimeout(5).andThen(new spit().withTimeout(.1)));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    shooterXbox.start().whileTrue(new eject().andThen(new flywheelSpit().withTimeout(.1)));
    shooterXbox.leftBumper().whileTrue(new intake()).onFalse(new spit().withTimeout(.1));
    shooterXbox.rightBumper().whileTrue(new flywheel().withTimeout(.5).andThen(new feed()));
    shooterXbox.a().onTrue(new raise());
    shooterXbox.b().onTrue(new lower());
  
    







    //Dpad for Climber

    shooterXbox.povUp().whileTrue(new up());
    shooterXbox.povUp().whileTrue(new up());
    shooterXbox.povUp().whileTrue(new up());
    shooterXbox.povDown().whileTrue(new down());
    shooterXbox.povDown().whileTrue(new down());
    shooterXbox.povDown().whileTrue(new down());
    shooterXbox.povRight().whileTrue(new right());
    shooterXbox.povLeft().whileTrue(new left());

    
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
    return autoChooser.getSelected();
    //"Nothing"
    //"Shoot Taxi Intake"
    //"Taxi Only"
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