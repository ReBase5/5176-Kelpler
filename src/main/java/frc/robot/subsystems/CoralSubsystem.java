// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
  private final SparkMax shooter = new SparkMax(24, MotorType.kBrushless);;
  private final SparkMax angleMotor = new SparkMax(22, MotorType.kBrushless);;
 
  private SparkClosedLoopController angleController = angleMotor.getClosedLoopController();

  private double coralAngle = 0;
  private RelativeEncoder angleEncoder = angleMotor.getEncoder();

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    shooter.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(10);
    rollerConfig.smartCurrentLimit(60);
    shooter.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig angleConfig = new SparkMaxConfig();
    angleConfig.idleMode(IdleMode.kBrake);
    angleConfig.closedLoop.pid(0.01, 0, 0.002);

    angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setCoralAngle(double coralAngle) {
    this.coralAngle = coralAngle;
    angleController.setReference(coralAngle, SparkBase.ControlType.kPosition);
  }

  public boolean isAtCoralAngle() {
    if((coralAngle - 1 < angleEncoder.getPosition()) && (angleEncoder.getPosition() < coralAngle + 1))
      return true;
    else 
      return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /** This is a method that makes the roller spin */
  public void runRoller(double forward, double reverse) {
    shooter.set(forward - reverse);
  }
}
