
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//button numbers on the controller:
// https://www.google.com/url?sa=i&url=https%3A%2F%2Fwww.chiefdelphi.com%2Ft%2Fhow-to-program-an-xbox-controller-to-drive-a-robot%2F131164&psig=AOvVaw28II86to-llZYujh--NhGp&ust=1759627474419000&source=images&cd=vfe&opi=89978449&ved=0CBkQjhxqFwoTCIC8oKaxiZADFQAAAAAdAAAAABAE
//IO means Input/Output
public class IO {
    //create xbox controller objects
    public XboxController driverXbox = new XboxController(0);
    public XboxController operatorXbox = new XboxController(1);

    JoystickButton shootButton = new JoystickButton(operatorXbox, 0);//XboxControl:            (set me! Use the link above)
    JoystickButton recieveButton = new JoystickButton(operatorXbox, 0); //XboxControl:         (set me! Use the link above)
    
    JoystickButton elevatorHighButton = new JoystickButton(operatorXbox, 1);//XboxControl:      A
    JoystickButton elevatorMediumButton = new JoystickButton(operatorXbox, 2);//XboxControl:    B
    JoystickButton elevatorLowButton = new JoystickButton(operatorXbox, 4);//XboxControl:       Y

    JoystickButton angleShootButton = new JoystickButton(operatorXbox, 0);//XboxControl:        (set me! Use the link above)   
    JoystickButton angleRecieveButton = new JoystickButton(operatorXbox, 0);//XboxControl:      (set me! Use the link above)

    JoystickButton deepClimbPushButton = new JoystickButton(operatorXbox, 0);//XboxControl:     (set me! Use the link above)
    JoystickButton deepClimbPullButton = new JoystickButton(operatorXbox, 0);//XboxControl:     (set me! Use the link above)

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
        angleRecieveButton.onTrue(KelplerCommands.setCoralAngleRecieve);

        // buttons to control climb motors
        deepClimbPushButton.onTrue(KelplerCommands.setDeepClimbPush);
        deepClimbPullButton.onTrue(KelplerCommands.setDeepClimbPull);
    }

}
