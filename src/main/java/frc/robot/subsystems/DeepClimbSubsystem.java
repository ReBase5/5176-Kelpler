// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeepClimbSubsystem extends SubsystemBase {
  
  
  //placeholder device IDs
  // Creating Motor controller objects
  private final TalonFX deepClimbLeader = new TalonFX(1);
  private final TalonFX deepClimbFollower = new TalonFX(2);

  private final PositionVoltage deepClimbPositionVoltage = new PositionVoltage(0);

  private double deepClimbRotations = 0;
  

    /** Creates a new DeepClimbSubsystem. */
    public DeepClimbSubsystem() {
            // reset to safe factory default parameters before setting new configuration
        deepClimbLeader.getConfigurator().apply(new TalonFXConfiguration());
        deepClimbFollower.getConfigurator().apply(new TalonFXConfiguration());

        /* Create and apply configuation for angle motor 
        ****** Still need to set PID values ******
        */
        TalonFXConfiguration deepClimbConfig = new TalonFXConfiguration();
        deepClimbConfig.Slot0.kP = 0; // An erro of 1 rotation results in a 0 V output
        deepClimbConfig.Slot0.kI = 0; // No output for integrate error
        deepClimbConfig.Slot0.kD = 0; // A velocity of 1 rotation results in a 0 V output
        // Peak output of 8 V
        deepClimbConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));
        deepClimbLeader.setNeutralMode(NeutralModeValue.Brake);

        /* Retry config apply up to 5 times, report if failure */
        StatusCode statusOne = StatusCode.StatusCodeNotInitialized;
        StatusCode statusTwo = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        statusOne = deepClimbLeader.getConfigurator().apply(deepClimbConfig);
        statusTwo = deepClimbFollower.getConfigurator().apply(deepClimbConfig);
        if (statusOne.isOK() && statusTwo.isOK()) break;
        }
        if (!statusOne.isOK()) {
            System.out.println("Could not apply configs to DeepClimb motor ONE, error code: " + statusOne.toString());
        }
        if (!statusOne.isOK()) {
            System.out.println("Could not apply configs to DeepClimb motor TWO, error code: " + statusTwo.toString());
        }

        /* Make sure we start at 0 */
        deepClimbLeader.setPosition(0);
        deepClimbFollower.setPosition(0);
    }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void setDeepClimbPosition(double deepClimbInterval) {
    // This method will be called once per scheduler run
    if(deepClimbRotations >= 0 && deepClimbRotations <= 100){
      deepClimbRotations += deepClimbInterval;
      if(deepClimbRotations >= 0 && deepClimbRotations <= 100){
        deepClimbLeader.setControl(deepClimbPositionVoltage.withPosition(deepClimbRotations));
        deepClimbFollower.setControl(deepClimbPositionVoltage.withPosition(deepClimbRotations * -1.0));
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
