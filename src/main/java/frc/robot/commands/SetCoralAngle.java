// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.testArm;
import frc.robot.subsystems.CoralSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralAngle extends Command {
  /** Creates a new SetCoralAngle. */
  private CoralSubsystem coral;
  private double coralAngleRotations;

  public SetCoralAngle() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
// not sure if coralSubsystem is how it should be named - it was previously caled armSubystem.
  public SetCoralAngle(CoralSubsystem coralSubsystem, double coralAngle) {
    this.coral = coralSubsystem;
    this.coralAngleRotations = coralAngle;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral);
  }

//*************** */ Made it to here in my progress**********************
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coral.setCoralAngle(coralAngleRotations);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
