// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
  static {
  System.out.println("created coral subsystem");
  }

  // Creating motor objects
  // *** Need to find the device id for the TalonFX ***
  private final TalonFX angleMotor = new TalonFX(0);
  private final SparkMax coralShooter = new SparkMax(27, MotorType.kBrushless);

  private double coralAngle = 0;

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    coralShooter.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(10);
    rollerConfig.smartCurrentLimit(60);
    coralShooter.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // reset to safe factory default parameters before setting new configuration
    angleMotor.getConfigurator().apply(new TalonFXConfiguration());

    /* Create and apply configuation for angle motor 
     ****** Still need to set PID values ******
    */
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();
    angleConfig.Slot0.kP = 0; // An erro of 1 rotation results in a 0 V output
    angleConfig.Slot0.kI = 0; // No output for integrate error
    angleConfig.Slot0.kD = 0; // A velocity of 1 rotation results in a 0 V output
    // Peak output of 8 V
    angleConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));
    angleMotor.setNeutralMode(NeutralModeValue.Brake);

    angleConfig.Slot1.kP = 0; // An error of 1 rotation results in 0 A output
    angleConfig.Slot1.kI = 0; // No output for integrated error
    angleConfig.Slot1.kD = 0; // A velocity of 1 rps results in 0 A output
    // Peak output of 120 A
    angleConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
      .withPeakReverseTorqueCurrent(Amps.of(-120));

      /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = angleMotor.getConfigurator().apply(angleConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    /* Make sure we start at 0 */
    angleMotor.setPosition(0);
  }
  

  // set the angle of the angle motor
  public void setCoralAngle(double coralAngle) {
    this.coralAngle = coralAngle;
    // need to create position voltage and position torque objects and
    // set them to desired rotations, but I don't think that is supposed
    // to happen in this file
  }
  
  public double getCoralAngle() {
    return coralAngle;
  }
  
  /* I don't think this is needed because positiondutycycle does this already from my understanding.
  public boolean isAtCoralAngle() {
    if((coralAngle - 1 < angleEncoder.getPosition()) && (angleEncoder.getPosition() < coralAngle + 1))
      return true;
    else 
      return false;
  }
*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CoralAngle", coralAngle);
  }
  /** This is a method that makes the roller spin */
  public void runRoller(double forward, double reverse) {
    coralShooter.set(forward - reverse);
  }
}