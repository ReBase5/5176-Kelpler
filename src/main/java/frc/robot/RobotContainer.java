// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CoralCommand;
import frc.robot.commands.SetAlgaeAngle;
import frc.robot.commands.SetCoralAngle;
import frc.robot.commands.SetElevatorPose;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  //Coral shooting subsystem created
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

  //Coral angle subsystem created

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final SetElevatorPose setElevatorPoseZero = new SetElevatorPose(elevatorSubsystem, 0);
  private final SetElevatorPose setElevatorPoseLowGoal = new SetElevatorPose(elevatorSubsystem, 60);
  private final SetElevatorPose setElevatorPoseMediumGoal = new SetElevatorPose(elevatorSubsystem, 140);
  private final SetElevatorPose setElevatorPoseRecieve = new SetElevatorPose(elevatorSubsystem, 160);
  private final SetElevatorPose setElevatorPoseHighGoal = new SetElevatorPose(elevatorSubsystem, 282);

  private final SetCoralAngle setCoralAngleZero = new SetCoralAngle(coralSubsystem, 0);
  private final SetCoralAngle setCoralAngleRecieve = new SetCoralAngle(coralSubsystem, 30);
  private final SetCoralAngle setCoralAngleShoot = new SetCoralAngle(coralSubsystem, 50);

  private final SetAlgaeAngle setAlgaeAngleUp = new SetAlgaeAngle(algaeSubsystem, 0);
  private final SetAlgaeAngle setAlgaeAngleDown = new SetAlgaeAngle(algaeSubsystem, 10);

//private  double forwardholder = 5.0;
 //private  double nullHolder = 0.0;
  //private final ShooterCommand shoot = new ShooterCommand(forwardholder, nullHolder, coralSubsystem);
  //private final ShooterCommand shoot = new ShooterCommand(operatorXbox.getLeftY(), 0, coralSubsystem);
  private final ShooterCommand shoot = new ShooterCommand(2, 0, coralSubsystem); //good one
  private final ShooterCommand stopShoot = new ShooterCommand(0, 0, coralSubsystem);
  
  //private final ShooterCommand shootBackward = new ShooterCommand(0.0, 5.0, coralSubsystem);

  //ivate final ShooterCommand activateShooter
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand("setAlgaeAngleUp", setAlgaeAngleUp);
    NamedCommands.registerCommand("setAlgaeAngleDown", setAlgaeAngleDown);
    NamedCommands.registerCommand("setCoralAngleRecieve", setCoralAngleRecieve);
    NamedCommands.registerCommand("setCoralAngleShoot", setCoralAngleShoot);
    NamedCommands.registerCommand("setElevatorPoseZero", setElevatorPoseZero);
    NamedCommands.registerCommand("setElevatorPoseLowGoal", setElevatorPoseLowGoal);
    NamedCommands.registerCommand("setElevatorPoseMediumGoal", setElevatorPoseMediumGoal);
    NamedCommands.registerCommand("setElevatorPoseRecieve", setElevatorPoseRecieve);
    NamedCommands.registerCommand("setElevatorPoseHighGoal", setElevatorPoseHighGoal);
    NamedCommands.registerCommand("shoot", shoot);
    //NamedCommands.registerCommand("shootBackward", shootBackward);

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    //NamedCommands.registerCommand("test", Commands.print("I EXIST"));
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
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driveDirectAngleKeyboard.driveToPose(() -> new Pose2d(new Translation2d(9, 3),
                                                            Rotation2d.fromDegrees(90)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5,
                                                                                     3)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(
                                                                         Math.toRadians(
                                                                             360),
                                                                         Math.toRadians(
                                                                             90))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());

      operatorXbox.a().onTrue(setElevatorPoseZero);
      operatorXbox.b().onTrue(setElevatorPoseLowGoal);
      operatorXbox.x().onTrue(setElevatorPoseHighGoal);
      operatorXbox.y().onTrue(setElevatorPoseMediumGoal);
      operatorXbox.back().onTrue(setElevatorPoseRecieve);

      operatorXbox.leftBumper().onTrue(setCoralAngleRecieve);
      operatorXbox.leftBumper().onFalse(setCoralAngleShoot);

      //operatorXbox.rightBumper().whileTrue(setAlgaeAngleUp);

      operatorXbox.start().onTrue(shoot);
      operatorXbox.start().onFalse(stopShoot);

      
      //if(operatorXbox.getLeftY() > 0) {
        //operatorXbox.leftStick().whileTrue(shoot);
      //}
      //else {
        //operatorXbox.leftStick().whileTrue(shootBackward);
      //}
      //operatorXbox.rightBumper().whileTrue(new CoralCommand(() -> CoralConstants.CORAL_EJECT_VALUE_FAST, () -> 0, coralSubsystem));
    }
    // Insert controller bindings for coral shooter
   //operatorXbox.rightBumper().whileTrue(new CoralCommand(() -> CoralConstants.CORAL_EJECT_VALUE_FAST, () -> 0, coralSubsystem));
   //operatorXbox.rightBumper().onFalse(new CoralCommand(() -> 0, () -> 0, coralSubsystem)); 
   //.whileTrue(coralSubsystem.runRoller(CoralConstants.CORAL_EJECT_VALUE_FAST, 0));
        //.whileTrue(new CoralCommand(() -> Constants.CoralConstants.CORAL_EJECT_VALUE_FAST, coralSubsystem));

  //operatorXbox.rightStick()
      //.whileTrue(coralSubsystem.runRoller(0, CoralConstants.CORAL_EJECT_VALUE_FAST));
        //.whileTrue(new CoralCommand(() -> CoralConstants.CORAL_EJECT_VALUE_SLOW, () -> 0, coralSubsystem));
        //.whileTrue(new CoralCommand(() -> Constants.CoralConstants.CORAL_EJECT_VALUE_SLOW, coralSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("MediumScoreintoStation");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
