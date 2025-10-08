
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//button numbers on the controller:
// https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.chiefdelphi.com%2Ft%2Fhow-to-program-an-xbox-controller-to-drive-a-robot%2F131164&psig=AOvVaw28II86to-llZYujh--NhGp&ust=1759627474419000&source=images&cd=vfe&opi=89978449&ved=0CBkQjhxqFwoTCIC8oKaxiZADFQAAAAAdAAAAABAE
//IO means Input/Output
public class IO {
    //create xbox controller objects
    public XboxController driverXbox = new XboxController(0);
    public XboxController operatorXbox = new XboxController(1);

    Trigger shootButton = new Trigger(() -> operatorXbox.getRightTriggerAxis() > 0.9);//XboxControl..........Right Trigger
    Trigger recieveButton = new Trigger(() -> operatorXbox.getLeftTriggerAxis() > 0.9); //XboxControl........Left Trigger
    
    JoystickButton elevatorHighButton = new JoystickButton(operatorXbox, 4);//XboxControl:......X
    JoystickButton elevatorMediumButton = new JoystickButton(operatorXbox, 3);//XboxControl.....Y
    JoystickButton elevatorLowButton = new JoystickButton(operatorXbox, 2);//XboxControl........B

    JoystickButton angleShootButton = new JoystickButton(operatorXbox, 6);//XboxControl.........Right Bumper   
    JoystickButton angleReceiveButton = new JoystickButton(operatorXbox, 5);//XboxControl.......Left Bumper
    Trigger angleBlockedReceiveButton = new Trigger(() -> operatorXbox.getPOV() == 180);//XboxControl........D-pad Left

    Trigger algaeL2Remove = new Trigger(() -> operatorXbox.getPOV() == 90);
    Trigger algaeL3Remove = new Trigger(() -> operatorXbox.getPOV() == 270);

    Trigger deepClimbPushButton = new Trigger(() -> driverXbox.getPOV() == 90);//XboxControl.................D-pad Up
    Trigger deepClimbPullButton = new Trigger(() -> driverXbox.getPOV() == 270);//XboxContro.................D-pad Down

    public IO() {

        // buttons to run shoot and receive rollers
        shootButton.onTrue(KelplerCommands.shootCoral);
        recieveButton.onTrue(KelplerCommands.recieveCoral);

        // buttons to run elevator height
        elevatorHighButton.onTrue(KelplerCommands.setElevatorHighGoal);
        elevatorMediumButton.onTrue(KelplerCommands.setElevatorMidGoal);
        elevatorLowButton.onTrue(KelplerCommands.setElevatorLowGoal);

        // buttons to set shoot and receive angles
        angleShootButton.onTrue(KelplerCommands.setCoralAngleShoot);
        angleReceiveButton.onTrue(KelplerCommands.setCoralAngleRecieve);

        // buttons to remove algae -- command groups, no need to set elevator height or coral angle
        algaeL2Remove.onTrue(KelplerCommands.removeL2Algae);
        algaeL3Remove.onTrue(KelplerCommands.removeL3Algae);

        // buttons to control climb motors
        deepClimbPushButton.onTrue(KelplerCommands.setDeepClimbPush);
        deepClimbPullButton.onTrue(KelplerCommands.setDeepClimbPull);
    }

}
