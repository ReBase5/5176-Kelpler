// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax motorA = new SparkMax(40, MotorType.kBrushless); //left motor
  // private SparkMax motorA = new SparkMax(33, MotorType.kBrushless);
  private SparkMax motorB = new SparkMax(42, MotorType.kBrushless); //right motor
  // private SparkMax motorB = new SparkMax(44, MotorType.kBrushless);

  private SparkClosedLoopController controllerA = motorA.getClosedLoopController();
  private SparkClosedLoopController controllerB = motorB.getClosedLoopController();

  private double setPoint = 0;

  private RelativeEncoder encoderA = motorA.getEncoder();
  private RelativeEncoder encoderB = motorB.getEncoder();
  

  /** Creates a new testArm. */
  public ElevatorSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kCoast);
    config.closedLoop.pid(0.001, 0, 0.08);
    config.inverted(true);

    motorA.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorB.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  public void setElevatorPose(double setPoint) {
    this.setPoint = setPoint;
    controllerA.setReference(setPoint, SparkBase.ControlType.kPosition);
    controllerB.setReference(setPoint, SparkBase.ControlType.kPosition);
  }

  public boolean isAtSetPoint() {
    if((setPoint - 1 < encoderA.getPosition()) && (encoderA.getPosition() < setPoint + 1))
      return true;
    else 
      return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmPoseA", encoderA.getPosition());
    SmartDashboard.putNumber("Arm setPoint", setPoint);
    SmartDashboard.putBoolean("ArmAtSetPoint", isAtSetPoint());
  }
}
