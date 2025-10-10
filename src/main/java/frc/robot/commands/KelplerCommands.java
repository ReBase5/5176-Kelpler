package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Constants;

// "() ->", or lambda function, allows to put in a method value where otherwise unable

public class KelplerCommands {
    //deep climb commands
    public static final Command setDeepClimbPush = new InstantCommand(() -> Robot.deepClimb.setDeepClimbPosition(Constants.DeepClimbConstants.DEEP_CLIMB_PUSH_INTERVAL), Robot.deepClimb);
    public static final Command setDeepClimbPull = new InstantCommand(() -> Robot.deepClimb.setDeepClimbPosition(-Constants.DeepClimbConstants.DEEP_CLIMB_PUSH_INTERVAL), Robot.deepClimb);

    //elevator commands
    public static final Command setElevatorLowGoal = new InstantCommand(
        () -> Robot.elevator.setElevatorPose(Constants.ElevatorConstants.ELEVATOR_LOW_GOAL), Robot.elevator);
    public static final Command setElevatorMidGoal = new InstantCommand(
        () -> Robot.elevator.setElevatorPose(Constants.ElevatorConstants.ELEVATOR_MID_GOAL), Robot.elevator);
    public static final Command setElevatorHighGoal = new InstantCommand(
        () -> Robot.elevator.setElevatorPose(Constants.ElevatorConstants.ELEVATOR_HIGH_GOAL), Robot.elevator);
        public static final Command setElevatorZeroGoal = new InstantCommand(
            () -> Robot.elevator.setElevatorPose(Constants.ElevatorConstants.ELEVATOR_ZERO_GOAL), Robot.elevator);

    //coral angle commands 
    //still need to set angles in the Constants file
    public static final Command setCoralAngleRecieve = new InstantCommand(
        () -> Robot.coralSubsystem.setCoralAngle(Constants.CoralConstants.CORAL_RECIEVE_ANGLE), Robot.coralSubsystem);
    public static final Command setCoralAngleRecieve2 = new InstantCommand(
        () -> Robot.coralSubsystem.setCoralAngle(Constants.CoralConstants.CORAL_RECIEVE_ANGLE2), Robot.coralSubsystem);
    public static final Command setCoralAngleShoot = new InstantCommand(
        () -> Robot.coralSubsystem.setCoralAngle(Constants.CoralConstants.CORAL_SHOOT_ANGLE), Robot.coralSubsystem);

    //coral shooter commands
    public static final Command shootCoral = new InstantCommand(()-> Robot.coralSubsystem.runRoller(0.3,0), Robot.coralSubsystem);
    public static final Command recieveCoral = new InstantCommand(()-> Robot.coralSubsystem.runRoller(0,0.3), Robot.coralSubsystem);
    public static final Command stopRollers = new InstantCommand(()-> Robot.coralSubsystem.runRoller(0,0), Robot.coralSubsystem);


    //to create a command group:
    public static final Command scoreLevelFour = new SequentialCommandGroup(
        new InstantCommand(() -> Robot.coralSubsystem.setCoralAngle(Constants.CoralConstants.CORAL_SHOOT_ANGLE), Robot.coralSubsystem), 
        new InstantCommand(() -> Robot.elevator.setElevatorPose(Constants.ElevatorConstants.ELEVATOR_HIGH_GOAL), Robot.elevator)
    );
    
    // check for increase in roller speed
    public static final Command removeL2Algae = new SequentialCommandGroup(
        new InstantCommand(() -> Robot.coralSubsystem.setCoralAngle(Constants.AlgaeConstants.CLEAR_ALGAE_ANGLE), Robot.coralSubsystem),
        new InstantCommand(() -> Robot.elevator.setElevatorPose(Constants.AlgaeConstants.ELEVATOR_CLEAR_ALGAE_L2), Robot.elevator),
        new RunCommand(() -> Robot.coralSubsystem.runRoller(0,0.3), Robot.coralSubsystem)
    );

    // check for increase in roller speed
    public static final Command removeL3Algae = new SequentialCommandGroup(
        new InstantCommand(() -> Robot.coralSubsystem.setCoralAngle(Constants.AlgaeConstants.CLEAR_ALGAE_ANGLE), Robot.coralSubsystem),
        new InstantCommand(() -> Robot.elevator.setElevatorPose(Constants.AlgaeConstants.ELEVATOR_CLEAR_ALGAE_L3), Robot.elevator),
        new RunCommand(() -> Robot.coralSubsystem.runRoller(0,0.3), Robot.coralSubsystem)
    );
    //many other cool possibilities with "WaitUntilCommand"
    //many other cool possibilities with limit switches
}
