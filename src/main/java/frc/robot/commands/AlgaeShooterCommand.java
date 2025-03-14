// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeShooterCommand extends Command {
  //private final DoubleSupplier forward;
  //private final DoubleSupplier reverse;
  private final double forward;
  private final double reverse;
  private final AlgaeSubsystem algaeSub;
 /** Creates a new ShooterCommand. */
  
  public AlgaeShooterCommand(
      //DoubleSupplier forward, DoubleSupplier reverse, CoralSubsystem coralSub){
        double forward, double reverse, AlgaeSubsystem algaeSub){
    this.forward = forward;
    this.reverse = reverse;
    this.algaeSub = algaeSub;

    addRequirements(this.algaeSub);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  // Run the roller motor at the desired speed
  algaeSub.runRoller(forward, reverse);

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