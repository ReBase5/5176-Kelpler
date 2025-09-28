// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPose extends Command {
  private ElevatorSubsystem elevator;
  private double setPoint;

  /** Creates a new SetelevatorPose. */
  public SetElevatorPose(ElevatorSubsystem elevatorSubsystem, double setPoint) {
    this.elevator = elevatorSubsystem;
    this.setPoint = setPoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setElevatorPose(setPoint);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isAtSetPoint();
  }
}
