// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax elevatorLeft;
  private SparkMax elevatorRight;
  private SparkMaxConfig elevatorLeftConfig;
  private SparkMaxConfig elevatorRightConfig;

  private double rampRate = 0.4; // was .2, doubled to try and observe effect

  public RelativeEncoder elevatorRightEncoder;

  public static double currentPosition; 

  public ElevatorSubsystem() {

    elevatorLeftConfig = new SparkMaxConfig();
    elevatorLeftConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(rampRate)
      .softLimit
        .forwardSoftLimit(60)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true); // TODO Confirm this slows initial ramp up? Make much longer?

    elevatorRightConfig = new SparkMaxConfig();
    elevatorRightConfig
      .smartCurrentLimit(40) 
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(rampRate) 
      .softLimit
        .forwardSoftLimit(0.0)
        .forwardSoftLimitEnabled(true) // TODO update to true after setting soft limits
        .reverseSoftLimit(-60.0)
        .reverseSoftLimitEnabled(true);
      
    elevatorLeft = new SparkMax(10, MotorType.kBrushless);
    elevatorLeft.configure(elevatorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorRight = new SparkMax(9, MotorType.kBrushless);
    elevatorRight.configure(elevatorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorRightEncoder = elevatorRight.getEncoder();
  }

  public void moveElevator(double speed) {
    elevatorRight.set(speed);
    elevatorLeft.set(-speed);
  }

  public boolean checkGroundPosition(){
    if(currentPosition > -10.0) // TODO finetune this value a bit
      return true;
    else
      return false;
  }

  @Override
  public void periodic() {
    currentPosition = elevatorRightEncoder.getPosition();
    // SmartDashboard.putNumber("Elevator Current Position: ", currentPosition);
    // SmartDashboard.putNumber("Elevator Setpoint", ElevatorCommand.elevatorSetpoint);
  }

}
