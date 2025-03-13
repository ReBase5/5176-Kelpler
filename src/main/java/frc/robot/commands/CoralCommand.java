//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import java.util.function.DoubleSupplier;

// Command to run the roller with joystick inputs
public class CoralCommand extends Command {
  private final DoubleSupplier forward;
  private final DoubleSupplier reverse;
  private final CoralSubsystem coralSubsystem;

  public CoralCommand(
      DoubleSupplier forward, DoubleSupplier reverse, CoralSubsystem coralSubsystem) {
    this.forward = forward;
    this.reverse = reverse;
    this.coralSubsystem = coralSubsystem;

    addRequirements(this.coralSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    // Run the roller motor at the desired speed
    coralSubsystem.runRoller(forward.getAsDouble(), reverse.getAsDouble());
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}
