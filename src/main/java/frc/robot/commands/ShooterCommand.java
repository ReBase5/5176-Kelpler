// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommand extends Command {
  private final DoubleSupplier forward;
  private final DoubleSupplier reverse;
  private final Coral coral;
 /** Creates a new ShooterCommand. */
  
  public ShooterCommand(
      DoubleSupplier forward, DoubleSupplier reverse, Coral coral) {
    this.forward = forward;
    this.reverse = reverse;
    this.coral = coral;

    addRequirements(this.coral);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  // Run the roller motor at the desired speed
  coral.runRoller(forward.getAsDouble(), reverse.getAsDouble());
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
