// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.MoveWristCommand;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax elevatorLeft;
  private SparkMax elevatorRight;
  private SparkMaxConfig elevatorLeftConfig;
  private SparkMaxConfig elevatorRightConfig;

  private double rampRate = 0.2;

  public RelativeEncoder elevatorRightEncoder;

  public static double currentPosition; 

  public ElevatorSubsystem() {

    elevatorLeftConfig = new SparkMaxConfig();
    elevatorLeftConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(rampRate); // TODO Confirm this slows initial ramp up?

    elevatorRightConfig = new SparkMaxConfig();
    elevatorRightConfig
      .smartCurrentLimit(40) 
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(rampRate) 
      .softLimit
        .forwardSoftLimit(0.0)
        .forwardSoftLimitEnabled(false) // TODO update to true after setting soft limits
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

  @Override
  public void periodic() {
    currentPosition = elevatorRightEncoder.getPosition();
    SmartDashboard.putNumber("Elevator Position: ", currentPosition);
    SmartDashboard.putNumber("Elevator Setpoint", ElevatorCommand.elevatorSetpoint);

    

    // SmartDashboard.putNumber("Selected Gampeiece", selectGamepiece);

    // if(RobotContainer.operatorController.back().getAsBoolean()){
    //   selectGamepiece = 1;
    // }
      
    // if(RobotContainer.operatorController.start().getAsBoolean()){
    //   selectGamepiece = 2;
    // }
  }

}
